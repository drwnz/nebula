// Copyright 2024 TIER IV, Inc.

#include "nebula_robosense/robosense_ros_wrapper.hpp"

#include "nebula_core_ros/parameter_descriptors.hpp"

#include <nebula_core_common/util/string_conversions.hpp>
#include <rclcpp/qos.hpp>

#include <robosense_msgs/msg/detail/robosense_info_packet__struct.hpp>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
#endif

namespace nebula::ros
{
RobosenseRosWrapper::RobosenseRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *sensor_cfg_ptr_);

  info_driver_.emplace(sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, sensor_cfg_ptr_);
  }

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  if (launch_hw_) {
    info_packets_pub_ =
      create_publisher<robosense_msgs::msg::RobosenseInfoPacket>("robosense_info_packets", 10);

    hw_interface_wrapper_->hw_interface()->register_scan_callback(
      std::bind(&RobosenseRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1));
    hw_interface_wrapper_->hw_interface()->register_info_callback(
      std::bind(&RobosenseRosWrapper::receive_info_packet_callback, this, std::placeholders::_1));
    stream_start();
  } else {
    packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "/robosense_packets", rclcpp::SensorDataQoS(),
      std::bind(&RobosenseRosWrapper::receive_scan_message_callback, this, std::placeholders::_1));
    info_packets_sub_ = create_subscription<robosense_msgs::msg::RobosenseInfoPacket>(
      "/robosense_info_packets", 10, [this](const robosense_msgs::msg::RobosenseInfoPacket & msg) {
        std::vector<uint8_t> raw_packet(msg.packet.data.begin(), msg.packet.data.end());
        receive_info_packet_callback(raw_packet);
      });
    RCLCPP_INFO_STREAM(
      get_logger(), "Hardware connection disabled, listening for packets on "
                      << packets_sub_->get_topic_name() << " and "
                      << info_packets_sub_->get_topic_name());

    // Initialize decoder wrapper with default calibration if in replay mode
    auto calib = drivers::RobosenseCalibrationConfiguration();
    if (sensor_cfg_ptr_->sensor_model == drivers::SensorModel::ROBOSENSE_EMX) {
      calib.set_channel_size(192);
    } else if (sensor_cfg_ptr_->sensor_model == drivers::SensorModel::ROBOSENSE_EM4) {
      calib.set_channel_size(520);
    } else if (sensor_cfg_ptr_->sensor_model == drivers::SensorModel::ROBOSENSE_E1) {
      calib.set_channel_size(128);
    } else {
      calib.set_channel_size(128);  // Fallback for others
    }

    auto calib_ptr =
      std::make_shared<const nebula::drivers::RobosenseCalibrationConfiguration>(std::move(calib));
    decoder_wrapper_.emplace(this, nullptr, sensor_cfg_ptr_, calib_ptr);
    RCLCPP_INFO_STREAM(
      get_logger(), "Initialized decoder wrapper for replay: " << decoder_wrapper_->status());
  }

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&RobosenseRosWrapper::on_parameter_change, this, std::placeholders::_1));
}

nebula::Status RobosenseRosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::RobosenseSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::sensor_model_from_string(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", param_read_write());
  config.return_mode = drivers::return_mode_from_string_robosense(_return_mode);

  config.host_ip = declare_parameter<std::string>("host_ip", param_read_only());
  config.sensor_ip = declare_parameter<std::string>("sensor_ip", param_read_only());
  config.data_port = declare_parameter<uint16_t>("data_port", param_read_only());
  config.gnss_port = declare_parameter<uint16_t>("gnss_port", param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", param_read_write());

  // scan_phase is only relevant for mechanical sensors that use angle-based frame splitting.
  // Directional sensors (E1, EM4, EMX) use packet-sequence-based splitting.
  if (
    config.sensor_model != drivers::SensorModel::ROBOSENSE_E1 &&
    config.sensor_model != drivers::SensorModel::ROBOSENSE_EM4 &&
    config.sensor_model != drivers::SensorModel::ROBOSENSE_EMX) {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.])";
    descriptor.floating_point_range = float_range(0, 360, 0.01);
    config.scan_phase = declare_parameter<double>("scan_phase", descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    descriptor.floating_point_range = float_range(0.01, 0.5, 0.01);
    config.dual_return_distance_threshold =
      declare_parameter<double>("dual_return_distance_threshold", descriptor);
  }

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::RobosenseSensorConfiguration>(config);
  return validate_and_set_config(new_cfg_ptr);
}

Status RobosenseRosWrapper::validate_and_set_config(
  std::shared_ptr<const drivers::RobosenseSensorConfiguration> & new_config)
{
  if (new_config->sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (new_config->return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (new_config->frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (hw_interface_wrapper_) {
    hw_interface_wrapper_->on_config_change(new_config);
  }
  if (hw_monitor_wrapper_) {
    hw_monitor_wrapper_->on_config_change(new_config);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->on_config_change(new_config);
  }

  sensor_cfg_ptr_ = new_config;
  return Status::OK;
}

void RobosenseRosWrapper::receive_scan_message_callback(
  nebula_msgs::msg::NebulaPackets::UniquePtr scan_msg)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Received NebulaPackets message with " << scan_msg->packets.size() << " packets.");

  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring received NebulaPackets. Launch with launch_hw:=false to enable NebulaPackets "
      "replay.");
    return;
  }

  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  for (auto & pkt : scan_msg->packets) {
    auto nebula_pkt_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>(pkt);

    decoder_wrapper_->process_cloud_packet(std::move(nebula_pkt_ptr));
  }
}

void RobosenseRosWrapper::receive_info_packet_callback(std::vector<uint8_t> & packet)
{
  if (!sensor_cfg_ptr_ || !info_driver_) {
    throw std::runtime_error(
      "Wrapper already receiving packets despite not being fully initialized yet.");
  }

  if (info_packets_pub_) {
    robosense_msgs::msg::RobosenseInfoPacket info_packet{};
    info_packet.packet.data = packet;
    info_packet.packet.stamp = now();
    info_packets_pub_->publish(info_packet);
  }

  auto status = info_driver_->decode_info_packet(packet);

  if (status != nebula::Status::OK) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *get_clock(), 1000, "Could not decode info packet: " << status);
    return;
  }

  if (!decoder_wrapper_) {
    auto new_cfg = *sensor_cfg_ptr_;

    // Check for configuration mismatches between driver settings and sensor-reported values.
    // Since RoboSense sensors cannot be configured from the driver, we warn the user
    // about any discrepancies so they can adjust either the driver config or the sensor
    // (via RSView or other vendor tools).
    auto sensor_return_mode = info_driver_->get_return_mode();
    static bool warned_return_mode_mismatch = false;
    if (
      !warned_return_mode_mismatch && sensor_return_mode != drivers::ReturnMode::UNKNOWN &&
      sensor_return_mode != new_cfg.return_mode) {
      warned_return_mode_mismatch = true;
      RCLCPP_WARN_STREAM(
        get_logger(), "Return mode mismatch: driver configured '"
                        << new_cfg.return_mode << "' but sensor reports '" << sensor_return_mode
                        << "'. Using sensor-reported value.");
    }

    // Check network config mismatches (sensor IP, host IP, ports)
    static bool warned_network_mismatch = false;
    if (!warned_network_mismatch) {
      auto sensor_info = info_driver_->get_sensor_info();
      std::string warnings;

      auto check_field =
        [&](const std::string & key, const std::string & driver_val, const std::string & label) {
          auto it = sensor_info.find(key);
          if (
            it != sensor_info.end() && !it->second.empty() && it->second != "0.0.0.0" &&
            it->second != "0" && it->second != driver_val) {
            warnings +=
              "  " + label + ": driver='" + driver_val + "', sensor='" + it->second + "'\n";
          }
        };

      check_field("sensor_ip", new_cfg.sensor_ip, "sensor_ip");
      check_field("dest_ip", new_cfg.host_ip, "host_ip");
      check_field("msop_dst_port", std::to_string(new_cfg.data_port), "data_port");
      check_field("difop_dst_port", std::to_string(new_cfg.gnss_port), "gnss_port");

      if (!warnings.empty()) {
        warned_network_mismatch = true;
        RCLCPP_WARN_STREAM(
          get_logger(), "Network config mismatch between driver and sensor:\n"
                          << warnings << "The driver may not receive data if these do not match.");
      }
    }

    if (sensor_return_mode != drivers::ReturnMode::UNKNOWN) {
      new_cfg.return_mode = sensor_return_mode;
    }
    new_cfg.use_sensor_time = info_driver_->get_sync_status();

    // Report time sync status once on startup
    static bool reported_sync_status = false;
    if (!reported_sync_status) {
      reported_sync_status = true;
      auto sensor_info = info_driver_->get_sensor_info();
      auto sync_mode_it = sensor_info.find("time_sync_mode");
      auto sync_status_it = sensor_info.find("sync_status");

      std::string sync_mode =
        (sync_mode_it != sensor_info.end()) ? sync_mode_it->second : "unknown";
      std::string sync_status =
        (sync_status_it != sensor_info.end()) ? sync_status_it->second : "unknown";

      if (new_cfg.use_sensor_time) {
        RCLCPP_INFO_STREAM(
          get_logger(), "Sensor time sync: mode='" << sync_mode << "', status='" << sync_status
                                                   << "'. Using sensor timestamps.");
      } else {
        RCLCPP_WARN_STREAM(
          get_logger(), "Sensor time sync: mode='"
                          << sync_mode << "', status='" << sync_status
                          << "'. Sync not active — using host time instead of sensor timestamps.");
      }
    }
    auto calib = info_driver_->get_sensor_calibration();
    calib.create_corrected_channels();

    auto new_cfg_ptr =
      std::make_shared<const nebula::drivers::RobosenseSensorConfiguration>(new_cfg);
    status = validate_and_set_config(new_cfg_ptr);

    if (status != nebula::Status::OK) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Invalid config from sensor (" << status << "): " << new_cfg);
      return;
    }

    auto calib_ptr =
      std::make_shared<const nebula::drivers::RobosenseCalibrationConfiguration>(std::move(calib));
    decoder_wrapper_.emplace(
      this, hw_interface_wrapper_ ? hw_interface_wrapper_->hw_interface() : nullptr,
      sensor_cfg_ptr_, calib_ptr);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Initialized decoder wrapper: " << decoder_wrapper_->status());
  }

  if (!hw_monitor_wrapper_) {
    return;
  }

  hw_monitor_wrapper_->diagnostics_callback(info_driver_->get_sensor_info());
}

Status RobosenseRosWrapper::get_status()
{
  return wrapper_status_;
}

Status RobosenseRosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }

  auto info_status = hw_interface_wrapper_->hw_interface()->info_interface_start();

  if (info_status != Status::OK) {
    return info_status;
  }

  return hw_interface_wrapper_->hw_interface()->sensor_interface_start();
}

rcl_interfaces::msg::SetParametersResult RobosenseRosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::RobosenseSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string _return_mode = "";
  bool got_any =
    get_param(p, "return_mode", _return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold);

  // scan_phase only applies to mechanical sensors
  if (
    new_cfg.sensor_model != drivers::SensorModel::ROBOSENSE_E1 &&
    new_cfg.sensor_model != drivers::SensorModel::ROBOSENSE_EM4 &&
    new_cfg.sensor_model != drivers::SensorModel::ROBOSENSE_EMX) {
    got_any |= get_param(p, "scan_phase", new_cfg.scan_phase);
  }

  // Currently, none of the wrappers have writeable parameters, so their update logic is not
  // implemented

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (_return_mode.length() > 0)
    new_cfg.return_mode = nebula::drivers::return_mode_from_string(_return_mode);

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::RobosenseSensorConfiguration>(new_cfg);
  auto status = validate_and_set_config(new_cfg_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Invalid configuration: " + util::to_string(status);
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

void RobosenseRosWrapper::receive_cloud_packet_callback(std::vector<uint8_t> & packet)
{
  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  const auto now = std::chrono::high_resolution_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(packet);

  decoder_wrapper_->process_cloud_packet(std::move(msg_ptr));
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseRosWrapper)
}  // namespace nebula::ros
