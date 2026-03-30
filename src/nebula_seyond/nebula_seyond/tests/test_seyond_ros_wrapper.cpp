// Copyright 2026 TIER IV, Inc.

#include <nebula_seyond/seyond_ros_wrapper.hpp>
#include <nebula_seyond_decoders/seyond_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::ros
{

class SeyondRosWrapperTestAccessor
{
public:
  static size_t packet_subscription_count(const SeyondRosWrapper & node)
  {
    if (!node.packets_pub_) {
      return 0;
    }
    return node.packets_pub_->get_subscription_count() +
           node.packets_pub_->get_intra_process_subscription_count();
  }

  static void inject_packet(
    SeyondRosWrapper & node, const std::vector<uint8_t> & packet,
    const builtin_interfaces::msg::Time & stamp)
  {
    node.process_packet(packet, stamp, true);
  }
};

}  // namespace nebula::ros

namespace
{

using nebula::drivers::SeyondDataPacket;

struct ReassemblyKey
{
  uint32_t src;
  uint32_t dst;
  uint16_t id;
  uint8_t protocol;

  bool operator<(const ReassemblyKey & other) const
  {
    return std::tie(src, dst, id, protocol) <
           std::tie(other.src, other.dst, other.id, other.protocol);
  }
};

struct FragmentAssembly
{
  bool saw_first_fragment{false};
  bool saw_last_fragment{false};
  uint16_t dest_port{0};
  size_t expected_udp_payload_size{0};
  std::vector<uint8_t> udp_payload{};
  std::vector<bool> received{};

  void ensure_capacity(size_t size)
  {
    if (udp_payload.size() < size) {
      udp_payload.resize(size);
      received.resize(size, false);
    }
  }

  bool is_complete() const
  {
    return saw_first_fragment && saw_last_fragment && expected_udp_payload_size > 0 &&
           udp_payload.size() >= expected_udp_payload_size &&
           std::all_of(
             received.begin(),
             received.begin() + static_cast<std::ptrdiff_t>(expected_udp_payload_size),
             [](bool value) { return value; });
  }
};

struct ReplayPacket
{
  builtin_interfaces::msg::Time stamp;
  std::vector<uint8_t> data;
  bool is_last_sub_frame{false};
};

rclcpp::NodeOptions make_options(const std::string & sensor_model)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("launch_hw", false);
  options.append_parameter_override("sensor_model", sensor_model);
  options.append_parameter_override("host_ip", "172.168.1.100");
  options.append_parameter_override("sensor_ip", "172.168.1.10");
  options.append_parameter_override("netmask", "255.255.255.0");
  options.append_parameter_override("gateway", "0.0.0.0");
  options.append_parameter_override("udp_port", 8010);
  options.append_parameter_override("udp_message_port", 8011);
  options.append_parameter_override("udp_status_port", 8012);
  options.append_parameter_override("setup_sensor", false);
  options.append_parameter_override("return_mode", "StrongestFurthest");
  options.append_parameter_override("reflectance_mode", "Reflectivity");
  options.append_parameter_override("time_sync", "PTP");
  options.append_parameter_override("frame_rate", 12.5);
  options.append_parameter_override("horizontal_roi", 30.0);
  options.append_parameter_override("vertical_roi", 5.0);
  return options;
}

std::optional<std::string> get_env(const char * name)
{
  const char * value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }
  return std::string(value);
}

builtin_interfaces::msg::Time to_builtin_time(const timeval & timestamp)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(timestamp.tv_sec);
  stamp.nanosec = static_cast<uint32_t>(timestamp.tv_usec * 1000);
  return stamp;
}

bool process_ipv4_packet(
  const pcap_pkthdr & header, const uint8_t * data, size_t size,
  std::map<ReassemblyKey, FragmentAssembly> & assemblies, uint16_t port_filter,
  std::vector<ReplayPacket> & replay_packets)
{
  constexpr size_t kEthernetHeaderSize = 14;
  if (size <= kEthernetHeaderSize) {
    return false;
  }

  const auto * ip_header = reinterpret_cast<const ip *>(data + kEthernetHeaderSize);
  if (ip_header->ip_v != 4 || ip_header->ip_p != IPPROTO_UDP) {
    return false;
  }

  const size_t ip_header_size = static_cast<size_t>(ip_header->ip_hl) * 4;
  const size_t total_ip_size = ntohs(ip_header->ip_len);
  if (
    ip_header_size < sizeof(ip) || kEthernetHeaderSize + total_ip_size > size ||
    total_ip_size < ip_header_size) {
    return false;
  }

  const size_t ip_payload_size = total_ip_size - ip_header_size;
  const uint8_t * ip_payload = reinterpret_cast<const uint8_t *>(ip_header) + ip_header_size;
  const uint16_t ip_off = ntohs(ip_header->ip_off);
  const bool more_fragments = (ip_off & IP_MF) != 0;
  const size_t fragment_offset = static_cast<size_t>(ip_off & IP_OFFMASK) * 8;

  ReassemblyKey key{
    ip_header->ip_src.s_addr, ip_header->ip_dst.s_addr, ntohs(ip_header->ip_id), ip_header->ip_p};
  auto & assembly = assemblies[key];

  if (fragment_offset == 0) {
    if (ip_payload_size < sizeof(udphdr)) {
      return false;
    }

    const auto * udp_header = reinterpret_cast<const udphdr *>(ip_payload);
    assembly.saw_first_fragment = true;
    assembly.dest_port = ntohs(udp_header->uh_dport);
    assembly.expected_udp_payload_size = ntohs(udp_header->uh_ulen) - sizeof(udphdr);
    assembly.ensure_capacity(assembly.expected_udp_payload_size);

    const size_t fragment_udp_data_size = ip_payload_size - sizeof(udphdr);
    const uint8_t * fragment_udp_data = ip_payload + sizeof(udphdr);
    std::copy(
      fragment_udp_data, fragment_udp_data + fragment_udp_data_size, assembly.udp_payload.begin());
    std::fill(
      assembly.received.begin(),
      assembly.received.begin() + static_cast<std::ptrdiff_t>(fragment_udp_data_size), true);
  } else {
    if (fragment_offset < sizeof(udphdr)) {
      return false;
    }

    const size_t udp_data_offset = fragment_offset - sizeof(udphdr);
    const size_t fragment_udp_data_size = ip_payload_size;
    assembly.ensure_capacity(udp_data_offset + fragment_udp_data_size);
    std::copy(
      ip_payload, ip_payload + fragment_udp_data_size,
      assembly.udp_payload.begin() + static_cast<std::ptrdiff_t>(udp_data_offset));
    std::fill(
      assembly.received.begin() + static_cast<std::ptrdiff_t>(udp_data_offset),
      assembly.received.begin() +
        static_cast<std::ptrdiff_t>(udp_data_offset + fragment_udp_data_size),
      true);
  }

  if (!more_fragments) {
    assembly.saw_last_fragment = true;
  }

  if (!assembly.is_complete()) {
    return false;
  }

  if (
    assembly.dest_port == port_filter && assembly.udp_payload.size() >= sizeof(SeyondDataPacket)) {
    const auto * packet = reinterpret_cast<const SeyondDataPacket *>(assembly.udp_payload.data());
    ReplayPacket replay_packet;
    replay_packet.stamp = to_builtin_time(header.ts);
    replay_packet.data = assembly.udp_payload;
    replay_packet.is_last_sub_frame = packet->is_last_sub_frame;
    replay_packets.emplace_back(std::move(replay_packet));
  }

  assemblies.erase(key);
  return true;
}

std::vector<ReplayPacket> load_replay_packets(const std::string & pcap_path, uint16_t port_filter)
{
  char errbuf[PCAP_ERRBUF_SIZE]{};
  pcap_t * pcap = pcap_open_offline(pcap_path.c_str(), errbuf);
  if (pcap == nullptr) {
    throw std::runtime_error("Failed to open pcap: " + std::string(errbuf));
  }

  std::map<ReassemblyKey, FragmentAssembly> assemblies;
  std::vector<ReplayPacket> replay_packets;
  pcap_pkthdr * header = nullptr;
  const u_char * data = nullptr;

  while (true) {
    const int rc = pcap_next_ex(pcap, &header, &data);
    if (rc == -2) {
      break;
    }
    if (rc < 0) {
      std::string error = pcap_geterr(pcap);
      pcap_close(pcap);
      throw std::runtime_error("pcap_next_ex failed: " + error);
    }
    if (rc == 0) {
      continue;
    }

    process_ipv4_packet(*header, data, header->caplen, assemblies, port_filter, replay_packets);
  }

  pcap_close(pcap);
  return replay_packets;
}

void ensure_rclcpp()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
}

}  // namespace

TEST(SeyondRosWrapper, ConstructsWithoutHardwareFalconK)
{
  ensure_rclcpp();
  auto node = std::make_shared<nebula::ros::SeyondRosWrapper>(make_options("FalconK"));
  ASSERT_NE(node, nullptr);
  rclcpp::shutdown();
}

TEST(SeyondRosWrapper, ConstructsWithoutHardwareRobinW)
{
  ensure_rclcpp();
  auto node = std::make_shared<nebula::ros::SeyondRosWrapper>(make_options("RobinW"));
  ASSERT_NE(node, nullptr);
  rclcpp::shutdown();
}

TEST(SeyondRosWrapper, ConstructsWithoutHardwareRobinE1X)
{
  ensure_rclcpp();
  auto node = std::make_shared<nebula::ros::SeyondRosWrapper>(make_options("RobinE1X"));
  ASSERT_NE(node, nullptr);
  rclcpp::shutdown();
}

TEST(SeyondRosWrapper, ConstructsWithoutHardwareHummingbirdD1)
{
  ensure_rclcpp();
  auto node = std::make_shared<nebula::ros::SeyondRosWrapper>(make_options("HummingbirdD1"));
  ASSERT_NE(node, nullptr);
  rclcpp::shutdown();
}

TEST(SeyondRosWrapper, PublishesAndReplaysRobinWPackets)
{
  const auto pcap_path = get_env("SEYOND_TEST_PCAP");
  const auto calibration_path = get_env("SEYOND_TEST_CALIBRATION");
  if (!pcap_path.has_value() || !calibration_path.has_value()) {
    GTEST_SKIP() << "SEYOND_TEST_PCAP and SEYOND_TEST_CALIBRATION are required";
  }

  ensure_rclcpp();

  auto options = make_options("RobinW");
  options.append_parameter_override("calibration_file", *calibration_path);
  auto wrapper = std::make_shared<nebula::ros::SeyondRosWrapper>(options);
  auto probe_node = std::make_shared<rclcpp::Node>("seyond_wrapper_probe");

  std::vector<size_t> packet_batch_sizes;
  std::vector<size_t> cloud_sizes;

  auto packets_sub = probe_node->create_subscription<nebula_msgs::msg::NebulaPackets>(
    "seyond_packets", rclcpp::SensorDataQoS(),
    [&packet_batch_sizes](nebula_msgs::msg::NebulaPackets::UniquePtr msg) {
      packet_batch_sizes.emplace_back(msg->packets.size());
    });
  auto cloud_sub = probe_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "seyond_points", rclcpp::SensorDataQoS(),
    [&cloud_sizes](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      const size_t point_count =
        msg->point_step == 0 ? 0 : static_cast<size_t>(msg->width) * msg->height;
      cloud_sizes.emplace_back(point_count);
    });

  (void)packets_sub;
  (void)cloud_sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(wrapper);
  executor.add_node(probe_node);

  const auto replay_packets = load_replay_packets(*pcap_path, 2373);
  ASSERT_FALSE(replay_packets.empty());

  const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < wait_deadline) {
    executor.spin_some();
    if (nebula::ros::SeyondRosWrapperTestAccessor::packet_subscription_count(*wrapper) >= 2U) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  size_t injected_packets = 0;
  for (const auto & replay_packet : replay_packets) {
    nebula::ros::SeyondRosWrapperTestAccessor::inject_packet(
      *wrapper, replay_packet.data, replay_packet.stamp);
    injected_packets++;

    for (int i = 0; i < 5; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (cloud_sizes.size() >= 3) {
      break;
    }
  }

  const auto settle_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < settle_deadline &&
         (packet_batch_sizes.size() < 3 || cloud_sizes.size() < 3)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_GE(injected_packets, 3U);
  ASSERT_GE(packet_batch_sizes.size(), 2U);
  ASSERT_GE(cloud_sizes.size(), 3U);

  EXPECT_GT(packet_batch_sizes[0], 0U);
  EXPECT_GT(packet_batch_sizes[1], 0U);

  EXPECT_EQ(cloud_sizes[0], 89157U);
  EXPECT_EQ(cloud_sizes[1], 89157U);
  EXPECT_EQ(cloud_sizes[2], 111200U);

  rclcpp::shutdown();
}
