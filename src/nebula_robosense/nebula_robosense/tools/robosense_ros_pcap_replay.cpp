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

#include "nebula_robosense/robosense_ros_wrapper.hpp"

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcap/pcap.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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
  std::filesystem::path wrapper_params_file;
  std::string packets_topic{"/robosense_packets"};
  std::string info_topic{"/robosense_info_packets"};
  std::string cloud_topic{"/robosense_points"};
  std::size_t max_clouds{10};
  uint16_t msop_port{0};
  std::set<uint16_t> info_ports;
  int settle_ms{2000};
};

struct UdpPacketView
{
  uint16_t src_port;
  uint16_t dst_port;
  const uint8_t * payload;
  std::size_t payload_size;
};

struct FieldOffsets
{
  std::size_t x{0};
  std::size_t y{0};
  std::size_t z{0};
  std::size_t intensity{0};
  uint8_t intensity_datatype{0};
  bool valid{false};
};

std::string usage(const char * program)
{
  std::ostringstream oss;
  oss << "Usage: " << program
      << " --pcap <path> --output-dir <dir> --msop-port <dst-port> --info-port <dst-port> "
         "[--info-port <dst-port> ...] [options]\n"
      << "Options:\n"
      << "  --wrapper-params-file <path>  Instantiate RobosenseRosWrapper in-process\n"
      << "  --packets-topic <name>  NebulaPackets topic (default: /robosense_packets)\n"
      << "  --info-topic <name>     RobosenseInfoPacket topic (default: /robosense_info_packets)\n"
      << "  --cloud-topic <name>    PointCloud2 output topic (default: /robosense_points)\n"
      << "  --max-clouds <n>        Number of clouds to save (default: 10)\n"
      << "  --settle-ms <ms>        Wait after playback for remaining clouds (default: 2000)\n";
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
    } else if (arg == "--wrapper-params-file") {
      options.wrapper_params_file = require_value("--wrapper-params-file");
    } else if (arg == "--packets-topic") {
      options.packets_topic = require_value("--packets-topic");
    } else if (arg == "--info-topic") {
      options.info_topic = require_value("--info-topic");
    } else if (arg == "--cloud-topic") {
      options.cloud_topic = require_value("--cloud-topic");
    } else if (arg == "--max-clouds") {
      options.max_clouds = static_cast<std::size_t>(std::stoul(require_value("--max-clouds")));
    } else if (arg == "--msop-port") {
      options.msop_port = static_cast<uint16_t>(std::stoul(require_value("--msop-port")));
    } else if (arg == "--info-port") {
      options.info_ports.insert(static_cast<uint16_t>(std::stoul(require_value("--info-port"))));
    } else if (arg == "--settle-ms") {
      options.settle_ms = std::stoi(require_value("--settle-ms"));
    } else if (arg == "-h" || arg == "--help") {
      throw std::runtime_error("");
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (
    options.pcap_path.empty() || options.output_dir.empty() || options.msop_port == 0 ||
    options.info_ports.empty()) {
    throw std::runtime_error(
      "--pcap, --output-dir, --msop-port and at least one --info-port are required");
  }

  return options;
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
    static_cast<uint16_t>((udp[0] << 8U) | udp[1]),
    static_cast<uint16_t>((udp[2] << 8U) | udp[3]),
    udp + kMinUdpHeaderSize,
    static_cast<std::size_t>(udp_length - kMinUdpHeaderSize)};
}

FieldOffsets get_field_offsets(const sensor_msgs::msg::PointCloud2 & msg)
{
  FieldOffsets offsets;
  for (const auto & field : msg.fields) {
    if (field.name == "x") offsets.x = field.offset;
    if (field.name == "y") offsets.y = field.offset;
    if (field.name == "z") offsets.z = field.offset;
    if (field.name == "intensity") {
      offsets.intensity = field.offset;
      offsets.intensity_datatype = field.datatype;
    }
  }
  offsets.valid = true;
  return offsets;
}

template <typename T>
T read_value(const std::vector<uint8_t> & data, std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

float read_intensity(
  const sensor_msgs::msg::PointCloud2 & msg, const FieldOffsets & offsets, std::size_t base_offset)
{
  const auto offset = base_offset + offsets.intensity;
  switch (offsets.intensity_datatype) {
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<float>(msg.data[offset]);
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<float>(read_value<uint16_t>(msg.data, offset));
    case sensor_msgs::msg::PointField::FLOAT32:
      return read_value<float>(msg.data, offset);
    default:
      return 0.0f;
  }
}

void write_pcd_ascii(const std::filesystem::path & output_path, const sensor_msgs::msg::PointCloud2 & msg)
{
  const auto offsets = get_field_offsets(msg);
  std::ofstream os(output_path);
  if (!os) {
    throw std::runtime_error("Failed to open output file: " + output_path.string());
  }

  const std::size_t point_count = static_cast<std::size_t>(msg.width) * msg.height;
  os << "# .PCD v0.7 - Point Cloud Data file format\n";
  os << "VERSION 0.7\n";
  os << "FIELDS x y z intensity\n";
  os << "SIZE 4 4 4 4\n";
  os << "TYPE F F F F\n";
  os << "COUNT 1 1 1 1\n";
  os << "WIDTH " << point_count << "\n";
  os << "HEIGHT 1\n";
  os << "VIEWPOINT 0 0 0 1 0 0 0\n";
  os << "POINTS " << point_count << "\n";
  os << "DATA ascii\n";
  os << std::fixed << std::setprecision(6);

  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t base_offset = i * msg.point_step;
    const float x = read_value<float>(msg.data, base_offset + offsets.x);
    const float y = read_value<float>(msg.data, base_offset + offsets.y);
    const float z = read_value<float>(msg.data, base_offset + offsets.z);
    const float intensity = read_intensity(msg, offsets, base_offset);
    os << x << " " << y << " " << z << " " << intensity << "\n";
  }
}

class ReplayHarness : public rclcpp::Node
{
public:
  explicit ReplayHarness(const CliOptions & options)
  : Node("robosense_ros_pcap_replay", rclcpp::NodeOptions().use_intra_process_comms(true)),
    options_(options)
  {
    packets_pub_ =
      create_publisher<nebula_msgs::msg::NebulaPackets>(options_.packets_topic, rclcpp::SensorDataQoS());
    info_pub_ =
      create_publisher<robosense_msgs::msg::RobosenseInfoPacket>(options_.info_topic, 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      options_.cloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cloud(*msg); });
  }

  std::size_t cloud_count() const { return cloud_count_; }

  void wait_for_subscriptions()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto packets_subscriptions =
        packets_pub_->get_subscription_count() + packets_pub_->get_intra_process_subscription_count();
      const auto info_subscriptions =
        info_pub_->get_subscription_count() + info_pub_->get_intra_process_subscription_count();
      if (packets_subscriptions > 0 && info_subscriptions > 0) {
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void publish_info_packets()
  {
    char errbuf[PCAP_ERRBUF_SIZE] = {};
    pcap_t * handle = pcap_open_offline(options_.pcap_path.c_str(), errbuf);
    if (handle == nullptr) {
      throw std::runtime_error("Failed to open pcap: " + std::string(errbuf));
    }

    pcap_pkthdr * header = nullptr;
    const uint8_t * packet = nullptr;
    while (pcap_next_ex(handle, &header, &packet) >= 0) {
      const auto udp = parse_udp_packet(*header, packet);
      if (!udp.has_value() || options_.info_ports.count(udp->dst_port) == 0) {
        continue;
      }
      robosense_msgs::msg::RobosenseInfoPacket msg;
      msg.packet.stamp = now();
      msg.packet.data.assign(udp->payload, udp->payload + udp->payload_size);
      info_pub_->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    pcap_close(handle);
  }

  void replay_packets(rclcpp::Executor & executor)
  {
    char errbuf[PCAP_ERRBUF_SIZE] = {};
    pcap_t * handle = pcap_open_offline(options_.pcap_path.c_str(), errbuf);
    if (handle == nullptr) {
      throw std::runtime_error("Failed to open pcap: " + std::string(errbuf));
    }

    pcap_pkthdr * header = nullptr;
    const uint8_t * packet = nullptr;
    while (cloud_count_ < options_.max_clouds && pcap_next_ex(handle, &header, &packet) >= 0) {
      const auto udp = parse_udp_packet(*header, packet);
      if (!udp.has_value() || udp->dst_port != options_.msop_port) {
        continue;
      }

      nebula_msgs::msg::NebulaPackets msg;
      msg.header.stamp = now();
      msg.packets.resize(1);
      msg.packets[0].stamp = msg.header.stamp;
      msg.packets[0].data.assign(udp->payload, udp->payload + udp->payload_size);
      packets_pub_->publish(msg);
      executor.spin_some();
    }
    pcap_close(handle);

    const auto settle_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(options_.settle_ms);
    while (cloud_count_ < options_.max_clouds && std::chrono::steady_clock::now() < settle_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (cloud_count_ >= options_.max_clouds) {
      return;
    }

    std::ostringstream stem;
    stem << "ros_cloud_" << std::setw(4) << std::setfill('0') << cloud_count_;
    write_pcd_ascii(options_.output_dir / (stem.str() + ".pcd"), msg);
    ++cloud_count_;
  }

  CliOptions options_;
  std::size_t cloud_count_{0};
  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_;
  rclcpp::Publisher<robosense_msgs::msg::RobosenseInfoPacket>::SharedPtr info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

std::shared_ptr<nebula::ros::RobosenseRosWrapper> make_wrapper(const CliOptions & options)
{
  if (options.wrapper_params_file.empty()) {
    return nullptr;
  }

  const auto info_port = *options.info_ports.begin();
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  node_options.arguments(
    {
      "--ros-args",
      "--params-file",
      options.wrapper_params_file.string(),
      "-p",
      "launch_hw:=false",
      "-p",
      "data_port:=" + std::to_string(options.msop_port),
      "-p",
      "gnss_port:=" + std::to_string(info_port),
      "-r",
      "/robosense_packets:=" + options.packets_topic,
      "-r",
      "/robosense_info_packets:=" + options.info_topic,
      "-r",
      "robosense_points:=" + options.cloud_topic,
    });
  return std::make_shared<nebula::ros::RobosenseRosWrapper>(node_options);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    const auto options = parse_args(argc, argv);
    std::filesystem::create_directories(options.output_dir);

    auto wrapper = make_wrapper(options);
    auto node = std::make_shared<ReplayHarness>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    if (wrapper) {
      executor.add_node(wrapper);
    }
    executor.add_node(node);

    node->wait_for_subscriptions();
    node->publish_info_packets();
    executor.spin_some();
    node->replay_packets(executor);

    std::cout << "Saved " << node->cloud_count() << " ROS cloud(s) to " << options.output_dir
              << std::endl;
    rclcpp::shutdown();
    return node->cloud_count() >= options.max_clouds ? 0 : 1;
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << std::endl;
    std::cerr << usage(argv[0]);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
