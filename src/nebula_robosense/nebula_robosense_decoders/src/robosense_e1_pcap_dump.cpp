// Copyright 2026 TIER IV, Inc.

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_robosense_common/robosense_common.hpp"
#include "nebula_robosense_decoders/robosense_driver.hpp"

#include <arpa/inet.h>
#include <pcap.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct EthernetHeader
{
  uint8_t dst[6];
  uint8_t src[6];
  uint16_t ether_type;
};

struct Ipv4Header
{
  uint8_t version_ihl;
  uint8_t dscp_ecn;
  uint16_t total_length;
  uint16_t identification;
  uint16_t flags_fragment_offset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t header_checksum;
  uint32_t src_addr;
  uint32_t dst_addr;
};

struct UdpHeader
{
  uint16_t src_port;
  uint16_t dst_port;
  uint16_t length;
  uint16_t checksum;
};

void usage(const char * program)
{
  std::cerr << "Usage: " << program
            << " <pcap_path> <output_csv> [sensor_ip=192.168.10.101] [udp_port=6699] [frames=1]\n";
}

std::vector<uint8_t> extract_udp_payload(
  const pcap_pkthdr & header, const uint8_t * packet, uint32_t expected_sensor_ip,
  uint16_t expected_port)
{
  if (header.caplen < sizeof(EthernetHeader) + sizeof(Ipv4Header) + sizeof(UdpHeader)) {
    return {};
  }

  const auto * ethernet = reinterpret_cast<const EthernetHeader *>(packet);
  if (ntohs(ethernet->ether_type) != 0x0800) {
    return {};
  }

  const auto * ip = reinterpret_cast<const Ipv4Header *>(packet + sizeof(EthernetHeader));
  const size_t ip_header_length = static_cast<size_t>(ip->version_ihl & 0x0F) * 4U;
  if (ip_header_length < sizeof(Ipv4Header)) {
    return {};
  }
  if (ip->protocol != 17) {
    return {};
  }
  if (ip->src_addr != expected_sensor_ip) {
    return {};
  }
  if (header.caplen < sizeof(EthernetHeader) + ip_header_length + sizeof(UdpHeader)) {
    return {};
  }

  const auto * udp =
    reinterpret_cast<const UdpHeader *>(packet + sizeof(EthernetHeader) + ip_header_length);
  if (ntohs(udp->src_port) != expected_port && ntohs(udp->dst_port) != expected_port) {
    return {};
  }

  const size_t udp_length = ntohs(udp->length);
  if (udp_length < sizeof(UdpHeader)) {
    return {};
  }

  const uint8_t * payload = reinterpret_cast<const uint8_t *>(udp) + sizeof(UdpHeader);
  const size_t payload_length = udp_length - sizeof(UdpHeader);
  if (header.caplen < (payload - packet) + payload_length) {
    return {};
  }

  return {payload, payload + payload_length};
}

void write_csv(
  const std::filesystem::path & output_path, const nebula::drivers::NebulaPointCloudPtr & cloud)
{
  std::ofstream out(output_path);
  if (!out) {
    throw std::runtime_error("failed to open output CSV: " + output_path.string());
  }

  out << "x,y,z,intensity,return_type,channel,azimuth,elevation,distance,time_stamp\n";
  out << std::fixed << std::setprecision(6);
  for (const auto & point : *cloud) {
    out << point.x << ',' << point.y << ',' << point.z << ','
        << static_cast<uint32_t>(point.intensity) << ',' << static_cast<uint32_t>(point.return_type)
        << ',' << point.channel << ',' << point.azimuth << ',' << point.elevation << ','
        << point.distance << ',' << point.time_stamp << '\n';
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 6) {
    usage(argv[0]);
    return 2;
  }

  const std::filesystem::path pcap_path = argv[1];
  const std::filesystem::path output_path = argv[2];
  const std::string sensor_ip_string = argc >= 4 ? argv[3] : "192.168.10.101";
  const uint16_t udp_port = argc >= 5 ? static_cast<uint16_t>(std::stoul(argv[4])) : 6699;
  const size_t max_frames = argc >= 6 ? static_cast<size_t>(std::stoul(argv[5])) : 1;

  uint32_t expected_sensor_ip{};
  if (inet_pton(AF_INET, sensor_ip_string.c_str(), &expected_sensor_ip) != 1) {
    std::cerr << "invalid sensor IP: " << sensor_ip_string << '\n';
    return 2;
  }

  auto sensor_configuration = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>();
  sensor_configuration->sensor_model = nebula::drivers::SensorModel::ROBOSENSE_E1;
  sensor_configuration->return_mode = nebula::drivers::ReturnMode::SINGLE_STRONGEST;
  sensor_configuration->frame_id = "robosense";

  auto calibration_configuration =
    std::make_shared<nebula::drivers::RobosenseCalibrationConfiguration>();
  nebula::drivers::RobosenseDriver driver(sensor_configuration, calibration_configuration);

  char errbuf[PCAP_ERRBUF_SIZE] = {};
  pcap_t * pcap = pcap_open_offline(pcap_path.c_str(), errbuf);
  if (pcap == nullptr) {
    std::cerr << "failed to open pcap: " << errbuf << '\n';
    return 1;
  }

  size_t packets_seen = 0;
  size_t udp_packets_seen = 0;
  std::vector<nebula::drivers::NebulaPointCloudPtr> clouds;
  std::vector<double> cloud_timestamps;

  while (true) {
    pcap_pkthdr * header = nullptr;
    const u_char * packet = nullptr;
    const int rc = pcap_next_ex(pcap, &header, &packet);
    if (rc == 1) {
      ++packets_seen;
      auto payload = extract_udp_payload(*header, packet, expected_sensor_ip, udp_port);
      if (payload.empty()) {
        continue;
      }
      ++udp_packets_seen;
      auto result = driver.parse_cloud_packet(payload);
      auto cloud = std::get<0>(result);
      auto cloud_timestamp = std::get<1>(result);
      if (cloud != nullptr) {
        clouds.push_back(std::make_shared<nebula::drivers::NebulaPointCloud>(*cloud));
        cloud_timestamps.push_back(cloud_timestamp);
        if (clouds.size() >= max_frames) {
          break;
        }
      }
    } else if (rc == -2) {
      break;
    } else {
      std::cerr << "pcap read error\n";
      pcap_close(pcap);
      return 1;
    }
  }

  pcap_close(pcap);

  if (clouds.empty()) {
    std::cerr << "no complete E1 frame decoded from pcap\n";
    return 1;
  }

  for (size_t frame_index = 0; frame_index < clouds.size(); ++frame_index) {
    const auto & cloud = clouds[frame_index];
    auto frame_output_path = output_path;
    if (max_frames > 1) {
      frame_output_path = output_path.parent_path() /
                          (output_path.stem().string() + "_" + std::to_string(frame_index) +
                           output_path.extension().string());
    }
    write_csv(frame_output_path, cloud);

    float min_x = cloud->front().x;
    float max_x = cloud->front().x;
    float min_y = cloud->front().y;
    float max_y = cloud->front().y;
    float min_z = cloud->front().z;
    float max_z = cloud->front().z;

    for (const auto & point : *cloud) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
    }

    std::cout << "frame=" << frame_index << '\n';
    std::cout << "points=" << cloud->size() << '\n';
    std::cout << "scan_timestamp_s=" << std::fixed << std::setprecision(9)
              << cloud_timestamps[frame_index] << '\n';
    std::cout << "bounds_x=[" << min_x << ", " << max_x << "]\n";
    std::cout << "bounds_y=[" << min_y << ", " << max_y << "]\n";
    std::cout << "bounds_z=[" << min_z << ", " << max_z << "]\n";
    std::cout << "csv=" << frame_output_path << '\n';
  }

  std::cout << "decoded_packets=" << udp_packets_seen << '\n';
  std::cout << "packets_scanned=" << packets_seen << '\n';
  std::cout << "frames=" << clouds.size() << '\n';

  return 0;
}
