// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nebula_robosense_decoders/offline/em4_pcap_calibration.hpp"

#include "nebula_robosense_decoders/robosense_info_driver.hpp"

#include <arpa/inet.h>
#include <pcap/pcap.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers
{
namespace
{

constexpr std::size_t kEthernetHeaderSize = 14;
constexpr uint16_t kEtherTypeIpv4 = 0x0800;
constexpr std::size_t kMinIpv4HeaderSize = 20;
constexpr std::size_t kMinUdpHeaderSize = 8;
constexpr uint8_t kIpProtoUdp = 17;
constexpr std::size_t kEm4Difop1Size = 1248;
constexpr std::size_t kEm4Difop2Size = 1162;

struct UdpPacketView
{
  uint32_t src_ip;
  uint32_t dst_ip;
  uint16_t src_port;
  uint16_t dst_port;
  const uint8_t * payload;
  std::size_t payload_size;
};

std::optional<uint32_t> parse_ipv4_address(const std::string & ip)
{
  if (ip.empty()) {
    return std::nullopt;
  }

  in_addr addr{};
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) {
    return std::nullopt;
  }
  return ntohl(addr.s_addr);
}

bool packet_matches_ip_filter(uint32_t packet_ip, const std::optional<uint32_t> & expected_ip)
{
  return !expected_ip.has_value() || packet_ip == *expected_ip;
}

bool packet_matches_sensor_filter(
  const UdpPacketView & udp, const RobosenseSensorConfiguration & sensor_config)
{
  const auto sensor_ip = parse_ipv4_address(sensor_config.sensor_ip);
  const auto host_ip = parse_ipv4_address(sensor_config.host_ip);
  const bool port_match =
    udp.src_port == sensor_config.gnss_port || udp.dst_port == sensor_config.gnss_port;
  const bool src_ip_match = packet_matches_ip_filter(udp.src_ip, sensor_ip);
  const bool dst_ip_match = packet_matches_ip_filter(udp.dst_ip, host_ip);

  return port_match && src_ip_match && dst_ip_match;
}

std::optional<UdpPacketView> parse_udp_packet(const pcap_pkthdr & header, const uint8_t * packet)
{
  if (header.caplen < kEthernetHeaderSize + kMinIpv4HeaderSize + kMinUdpHeaderSize) {
    return std::nullopt;
  }

  const auto * ethernet = packet;
  const uint16_t ether_type = static_cast<uint16_t>((ethernet[12] << 8U) | ethernet[13]);
  if (ether_type != kEtherTypeIpv4) {
    return std::nullopt;
  }

  const auto * ipv4 = packet + kEthernetHeaderSize;
  const uint8_t version = ipv4[0] >> 4U;
  const uint8_t ihl_words = ipv4[0] & 0x0FU;
  const std::size_t ip_header_size = static_cast<std::size_t>(ihl_words) * 4U;
  if (version != 4 || ip_header_size < kMinIpv4HeaderSize) {
    return std::nullopt;
  }
  if (header.caplen < kEthernetHeaderSize + ip_header_size + kMinUdpHeaderSize) {
    return std::nullopt;
  }
  if (ipv4[9] != kIpProtoUdp) {
    return std::nullopt;
  }

  const auto * udp = ipv4 + ip_header_size;
  const uint16_t udp_length = static_cast<uint16_t>((udp[4] << 8U) | udp[5]);
  if (udp_length < kMinUdpHeaderSize) {
    return std::nullopt;
  }

  const std::size_t available_udp_size = header.caplen - kEthernetHeaderSize - ip_header_size;
  if (available_udp_size < udp_length) {
    return std::nullopt;
  }

  return UdpPacketView{
    static_cast<uint32_t>((ipv4[12] << 24U) | (ipv4[13] << 16U) | (ipv4[14] << 8U) | ipv4[15]),
    static_cast<uint32_t>((ipv4[16] << 24U) | (ipv4[17] << 16U) | (ipv4[18] << 8U) | ipv4[19]),
    static_cast<uint16_t>((udp[0] << 8U) | udp[1]),
    static_cast<uint16_t>((udp[2] << 8U) | udp[3]),
    udp + kMinUdpHeaderSize,
    static_cast<std::size_t>(udp_length - kMinUdpHeaderSize)};
}

}  // namespace

std::shared_ptr<RobosenseCalibrationConfiguration> load_em4_calibration_from_pcap(
  const std::string & pcap_path, const RobosenseSensorConfiguration & sensor_config)
{
  if (sensor_config.sensor_model != SensorModel::ROBOSENSE_EM4) {
    throw std::runtime_error("EM4 PCAP calibration loader only supports sensor_model=EM4");
  }
  if (sensor_config.gnss_port == 0) {
    throw std::runtime_error("EM4 PCAP calibration loader requires a non-zero gnss_port");
  }

  auto sensor_cfg_ptr = std::make_shared<RobosenseSensorConfiguration>(sensor_config);
  RobosenseInfoDriver info_driver(sensor_cfg_ptr);

  char errbuf[PCAP_ERRBUF_SIZE] = {};
  pcap_t * handle = pcap_open_offline(pcap_path.c_str(), errbuf);
  if (handle == nullptr) {
    throw std::runtime_error("Failed to open pcap: " + std::string(errbuf));
  }

  const int linktype = pcap_datalink(handle);
  if (linktype != DLT_EN10MB) {
    pcap_close(handle);
    throw std::runtime_error("Unsupported link type in pcap");
  }

  bool saw_matching_info_stream = false;
  bool got_difop1 = false;
  bool got_difop2 = false;

  pcap_pkthdr * header = nullptr;
  const uint8_t * packet = nullptr;
  while (pcap_next_ex(handle, &header, &packet) >= 0) {
    const auto udp = parse_udp_packet(*header, packet);
    if (!udp.has_value() || !packet_matches_sensor_filter(*udp, sensor_config)) {
      continue;
    }

    saw_matching_info_stream = true;
    if (udp->payload_size != kEm4Difop1Size && udp->payload_size != kEm4Difop2Size) {
      continue;
    }

    std::vector<uint8_t> payload(udp->payload, udp->payload + udp->payload_size);
    if (info_driver.decode_info_packet(payload) != Status::OK) {
      continue;
    }

    got_difop1 = got_difop1 || udp->payload_size == kEm4Difop1Size;
    got_difop2 = got_difop2 || udp->payload_size == kEm4Difop2Size;
    if (got_difop2) {
      break;
    }
  }

  pcap_close(handle);

  if (!saw_matching_info_stream) {
    std::ostringstream oss;
    oss << "No EM4 info traffic matched gnss_port=" << sensor_config.gnss_port;
    if (!sensor_config.sensor_ip.empty()) {
      oss << ", sensor_ip=" << sensor_config.sensor_ip;
    }
    if (!sensor_config.host_ip.empty()) {
      oss << ", host_ip=" << sensor_config.host_ip;
    }
    throw std::runtime_error(oss.str());
  }
  if (!got_difop1 && !got_difop2) {
    throw std::runtime_error("No EM4 DIFOP packets were found in the matching info stream");
  }
  if (!got_difop2) {
    std::ostringstream oss;
    oss << "Partial EM4 info stream found in pcap";
    if (got_difop1) {
      oss << " (DIFOP1 present, DIFOP2 missing)";
    }
    throw std::runtime_error(oss.str());
  }

  auto calibration =
    std::make_shared<RobosenseCalibrationConfiguration>(info_driver.get_sensor_calibration());
  calibration->create_corrected_channels();
  return calibration;
}

}  // namespace nebula::drivers
