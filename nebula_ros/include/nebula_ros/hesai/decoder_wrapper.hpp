// Copyright 2024 TIER IV, Inc.
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

#pragma once

#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"
#include "nebula_ros/common/diagnostics/rate_bound_status.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>

#include <memory>
#include <mutex>

namespace nebula::ros
{
class HesaiDecoderWrapper
{
public:
  HesaiDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & calibration,
    diagnostic_updater::Updater & diagnostic_updater, bool publish_packets);

  void process_cloud_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config);

  void on_calibration_change(
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> &
      new_calibration);

  nebula::Status status();

private:
  void publish_cloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  static custom_diagnostic_tasks::RateBoundStatus make_rate_bound_status(
    uint16_t rpm, rclcpp::Node & node)
  {
    double nominal_rate_hz = drivers::rpm2hz(rpm);

    double min_ok_hz =
      node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_ok.min_hz");
    double max_ok_hz =
      node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_ok.max_hz");
    double min_warn_hz =
      node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_warn.min_hz");
    double max_warn_hz =
      node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_warn.max_hz");

    // Warn if misconfigured. Since this is not a critical error, continue operation.
    if (nominal_rate_hz < min_ok_hz || nominal_rate_hz > max_ok_hz) {
      RCLCPP_WARN(
        node.get_logger(),
        "The configured sensor framerate (%d RPM = %.2f Hz) is outside the configured framerate "
        "bounds: %.2f - %.2f. Please check the sensor configuration.",
        rpm, nominal_rate_hz, min_ok_hz, max_ok_hz);
    }

    custom_diagnostic_tasks::RateBoundStatusParam ok_params(min_ok_hz, max_ok_hz);
    custom_diagnostic_tasks::RateBoundStatusParam warn_params(min_warn_hz, max_warn_hz);
    return {&node, ok_params, warn_params};
  }

  nebula::Status status_;
  rclcpp::Logger logger_;
  rclcpp::Node & parent_node_;

  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_cfg_;
  std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> calibration_cfg_ptr_{};

  std::shared_ptr<drivers::HesaiDriver> driver_ptr_{};
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr packets_pub_{};
  pandar_msgs::msg::PandarScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};

  custom_diagnostic_tasks::RateBoundStatus publish_diagnostic_;

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};
}  // namespace nebula::ros
