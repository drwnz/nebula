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

#ifndef NEBULA_SEYOND_ROS_WRAPPER_HPP
#define NEBULA_SEYOND_ROS_WRAPPER_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_seyond/hw_interface_wrapper.hpp>
#include <nebula_seyond/hw_monitor_wrapper.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>
#include <nebula_seyond_hw_interfaces/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula::ros
{

class SeyondRosWrapperTestAccessor;

class SeyondRosWrapper : public rclcpp::Node
{
public:
  explicit SeyondRosWrapper(const rclcpp::NodeOptions & options);

private:
  friend class SeyondRosWrapperTestAccessor;

  void receive_scan_message_callback(nebula_msgs::msg::NebulaPackets::UniquePtr scan_msg);
  void receive_packet_callback(
    std::vector<uint8_t> & packet,
    const nebula::drivers::connections::UdpSocket::RxMetadata & metadata);
  void process_packet(
    const std::vector<uint8_t> & packet, const builtin_interfaces::msg::Time & stamp,
    bool collect_for_publish);
  void publish_cloud(nebula::drivers::NebulaPointCloudPtr cloud, uint64_t base_timestamp_ns);

  void declare_parameters();
  void get_parameters();

  std::shared_ptr<nebula::drivers::SeyondHwInterface> hw_interface_;
  std::unique_ptr<SeyondHwInterfaceWrapper> hw_interface_wrapper_;
  std::unique_ptr<SeyondHwMonitorWrapper> hw_monitor_wrapper_;
  std::unique_ptr<nebula::drivers::SeyondDecoder> decoder_;
  diagnostic_updater::Updater diagnostic_updater_;
  nebula::drivers::SeyondSensorConfiguration config_;
  bool launch_hw_{true};
  std::string calibration_file_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_;
  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_;
  nebula_msgs::msg::NebulaPackets::UniquePtr current_scan_msg_;
};

}  // namespace nebula::ros

#endif  // NEBULA_SEYOND_ROS_WRAPPER_HPP
