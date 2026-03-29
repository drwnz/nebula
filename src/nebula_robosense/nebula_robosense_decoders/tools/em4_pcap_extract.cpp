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

#include "nebula_core_common/point_types.hpp"
#include "nebula_robosense_common/robosense_common.hpp"
#include "nebula_robosense_decoders/robosense_driver.hpp"
#include "nebula_robosense_decoders/robosense_info_driver.hpp"

#include <pcap/pcap.h>
#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace
{

constexpr std::size_t kEthernetHeaderSize = 14;
constexpr uint16_t kEtherTypeIpv4 = 0x0800;
constexpr std::size_t kMinIpv4HeaderSize = 20;
constexpr std::size_t kMinUdpHeaderSize = 8;
constexpr uint8_t kIpProtoUdp = 17;

struct CliOptions
{
  std::filesystem::path pcap_path;
  std::filesystem::path output_dir;
  std::size_t max_frames{10};
  uint16_t msop_port{6699};
  uint16_t difop_port{7788};
  bool keep_empty_frames{false};
};

struct UdpPacketView
{
  uint16_t src_port;
  uint16_t dst_port;
  const uint8_t * payload;
  std::size_t payload_size;
};

std::string usage(const char * program)
{
  std::ostringstream oss;
  oss << "Usage: " << program << " --pcap <path> --output-dir <dir> [options]\n"
      << "Options:\n"
      << "  --max-frames <n>     Number of completed scans to save (default: 10)\n"
      << "  --msop-port <port>   MSOP UDP port (default: 6699)\n"
      << "  --difop-port <port>  DIFOP UDP port (default: 7788)\n"
      << "  --keep-empty-frames  Save frames even if they contain no points\n";
  return oss.str();
}

CliOptions parse_args(int argc, char ** argv)
{
  CliOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto require_value = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };

    if (arg == "--pcap") {
      options.pcap_path = require_value("--pcap");
    } else if (arg == "--output-dir") {
      options.output_dir = require_value("--output-dir");
    } else if (arg == "--max-frames") {
      options.max_frames = static_cast<std::size_t>(std::stoul(require_value("--max-frames")));
    } else if (arg == "--msop-port") {
      options.msop_port = static_cast<uint16_t>(std::stoul(require_value("--msop-port")));
    } else if (arg == "--difop-port") {
      options.difop_port = static_cast<uint16_t>(std::stoul(require_value("--difop-port")));
    } else if (arg == "--keep-empty-frames") {
      options.keep_empty_frames = true;
    } else if (arg == "-h" || arg == "--help") {
      throw std::runtime_error("");
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (options.pcap_path.empty() || options.output_dir.empty()) {
    throw std::runtime_error("Both --pcap and --output-dir are required");
  }
  if (options.max_frames == 0) {
    throw std::runtime_error("--max-frames must be greater than zero");
  }

  return options;
}

std::optional<UdpPacketView> parse_udp_packet(const pcap_pkthdr & header, const uint8_t * packet)
{
  if (header.caplen < kEthernetHeaderSize + kMinIpv4HeaderSize + kMinUdpHeaderSize) {
    return std::nullopt;
  }

  const auto * ethernet = packet;
  const uint16_t ether_type =
    static_cast<uint16_t>((ethernet[12] << 8U) | ethernet[13]);
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
  const uint16_t udp_length =
    static_cast<uint16_t>((udp[4] << 8U) | udp[5]);
  if (udp_length < kMinUdpHeaderSize) {
    return std::nullopt;
  }

  const std::size_t available_udp_size = header.caplen - kEthernetHeaderSize - ip_header_size;
  if (available_udp_size < udp_length) {
    return std::nullopt;
  }

  return UdpPacketView{
    static_cast<uint16_t>((udp[0] << 8U) | udp[1]),
    static_cast<uint16_t>((udp[2] << 8U) | udp[3]),
    udp + kMinUdpHeaderSize,
    static_cast<std::size_t>(udp_length - kMinUdpHeaderSize)};
}

void write_pcd_ascii(
  const std::filesystem::path & output_path, const nebula::drivers::NebulaPointCloud & cloud)
{
  std::ofstream os(output_path);
  if (!os) {
    throw std::runtime_error("Failed to open output file: " + output_path.string());
  }

  os << "# .PCD v0.7 - Point Cloud Data file format\n";
  os << "VERSION 0.7\n";
  os << "FIELDS x y z intensity\n";
  os << "SIZE 4 4 4 4\n";
  os << "TYPE F F F U\n";
  os << "COUNT 1 1 1 1\n";
  os << "WIDTH " << cloud.size() << "\n";
  os << "HEIGHT 1\n";
  os << "VIEWPOINT 0 0 0 1 0 0 0\n";
  os << "POINTS " << cloud.size() << "\n";
  os << "DATA ascii\n";
  os << std::fixed << std::setprecision(6);

  for (const auto & point : cloud) {
    os << point.x << " " << point.y << " " << point.z << " "
       << static_cast<unsigned int>(point.intensity) << "\n";
  }
}

std::shared_ptr<const nebula::drivers::RobosenseCalibrationConfiguration> load_em4_calibration(
  const CliOptions & options)
{
  auto sensor_config = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>();
  sensor_config->sensor_model = nebula::drivers::SensorModel::ROBOSENSE_EM4;
  sensor_config->return_mode = nebula::drivers::ReturnMode::SINGLE_STRONGEST;

  nebula::drivers::RobosenseInfoDriver info_driver(sensor_config);

  char errbuf[PCAP_ERRBUF_SIZE] = {};
  pcap_t * handle = pcap_open_offline(options.pcap_path.c_str(), errbuf);
  if (handle == nullptr) {
    throw std::runtime_error("Failed to open pcap: " + std::string(errbuf));
  }

  const int linktype = pcap_datalink(handle);
  if (linktype != DLT_EN10MB) {
    pcap_close(handle);
    throw std::runtime_error("Unsupported link type in pcap");
  }

  bool got_difop1 = false;
  bool got_difop2 = false;

  pcap_pkthdr * header = nullptr;
  const uint8_t * packet = nullptr;
  while (pcap_next_ex(handle, &header, &packet) >= 0) {
    const auto udp = parse_udp_packet(*header, packet);
    if (!udp.has_value()) {
      continue;
    }
    if (udp->payload_size != 1248 && udp->payload_size != 1162) {
      continue;
    }

    std::vector<uint8_t> payload(udp->payload, udp->payload + udp->payload_size);
    if (info_driver.decode_info_packet(payload) != nebula::Status::OK) {
      continue;
    }

    got_difop1 = got_difop1 || udp->payload_size == 1248;
    got_difop2 = got_difop2 || udp->payload_size == 1162;
    if (got_difop2) {
      break;
    }
  }

  pcap_close(handle);

  if (!got_difop2) {
    throw std::runtime_error("Did not find an EM4 DIFOP2 calibration packet in pcap");
  }

  auto calibration = info_driver.get_sensor_calibration();
  calibration.create_corrected_channels();
  return std::make_shared<const nebula::drivers::RobosenseCalibrationConfiguration>(
    std::move(calibration));
}

std::size_t extract_em4_frames(const CliOptions & options)
{
  auto sensor_config = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>();
  sensor_config->sensor_model = nebula::drivers::SensorModel::ROBOSENSE_EM4;
  sensor_config->return_mode = nebula::drivers::ReturnMode::SINGLE_STRONGEST;
  sensor_config->frame_id = "robosense";
  sensor_config->data_port = options.msop_port;
  sensor_config->gnss_port = options.difop_port;

  auto calibration = load_em4_calibration(options);
  nebula::drivers::RobosenseDriver driver(sensor_config, calibration);

  char errbuf[PCAP_ERRBUF_SIZE] = {};
  pcap_t * handle = pcap_open_offline(options.pcap_path.c_str(), errbuf);
  if (handle == nullptr) {
    throw std::runtime_error("Failed to open pcap: " + std::string(errbuf));
  }

  const int linktype = pcap_datalink(handle);
  if (linktype != DLT_EN10MB) {
    pcap_close(handle);
    throw std::runtime_error("Unsupported link type in pcap");
  }

  std::filesystem::create_directories(options.output_dir);

  std::size_t saved_frames = 0;
  pcap_pkthdr * header = nullptr;
  const uint8_t * packet = nullptr;
  while (saved_frames < options.max_frames && pcap_next_ex(handle, &header, &packet) >= 0) {
    const auto udp = parse_udp_packet(*header, packet);
    if (!udp.has_value()) {
      continue;
    }
    if (udp->src_port != options.msop_port && udp->dst_port != options.msop_port) {
      continue;
    }

    std::vector<uint8_t> payload(udp->payload, udp->payload + udp->payload_size);
    auto [cloud, timestamp_s] = driver.parse_cloud_packet(payload);
    if (cloud == nullptr) {
      continue;
    }
    if (cloud->empty() && !options.keep_empty_frames) {
      continue;
    }

    std::ostringstream stem;
    stem << "nebula_frame_" << std::setw(4) << std::setfill('0') << saved_frames
         << "_ts_" << std::fixed << std::setprecision(6) << timestamp_s;

    write_pcd_ascii(options.output_dir / (stem.str() + ".pcd"), *cloud);
    ++saved_frames;
  }

  pcap_close(handle);
  return saved_frames;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    const auto options = parse_args(argc, argv);
    const auto saved_frames = extract_em4_frames(options);
    std::cout << "Saved " << saved_frames << " frame(s) to " << options.output_dir << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << std::endl;
    std::cerr << usage(argv[0]);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
