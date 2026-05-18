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

#pragma once

#include "nebula_robosense_common/robosense_common.hpp"
#include "nebula_robosense_decoders/decoders/robosense_packet.hpp"
#include "nebula_robosense_decoders/decoders/robosense_scan_decoder.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor_directional.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::drivers
{

/// @brief Decoder for directional Robosense sensors (E1, EMX, EM4) that use packet-sequence-based
/// frame splitting and either direction vectors or DIFOP2 calibration for point coordinates.
/// @tparam SensorT The sensor definition type
template <typename T, bool HasCustom>
struct AngleCorrectorFactory
{
  static std::shared_ptr<T> create(
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & /*calib*/)
  {
    return nullptr;
  }
};

template <typename T>
struct AngleCorrectorFactory<T, true>
{
  static std::shared_ptr<T> create(
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & calib)
  {
    return std::make_shared<T>(calib);
  }
};

template <typename SensorT>
class RobosenseDecoderDirectional : public RobosenseScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::RobosenseSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;

  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_{0};
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_{0};
  /// @brief Whether a full scan has been processed
  bool has_scanned_{false};

  /// @brief The previous packet sequence number, used for frame boundary detection
  uint16_t prev_pkt_seq_{0};
  uint16_t safe_seq_min_{0};
  uint16_t safe_seq_max_{10};
  uint16_t max_pkt_seq_{0};
  bool seq_looped_{false};

  /// @brief Angle corrector
  std::shared_ptr<typename SensorT::angle_corrector_t> angle_corrector_;

  /// @brief Distance resolution in meters (5mm for E1/EMX/EM4)
  static constexpr float distance_resolution_m_ = 0.005f;

  /// @brief Base value for direction vector normalization (2^15)
  static constexpr float vector_base_ = 32768.0f;

  rclcpp::Logger logger_;

  /// @brief Validates and parses MsopPacket. Currently only checks size, not checksums etc.
  /// @param msop_packet The incoming MsopPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & msop_packet)
  {
    if (msop_packet.size() < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch: " << msop_packet.size() << " | Expected at least: "
                                          << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, msop_packet.data(), sizeof(typename SensorT::packet_t))) {
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Check if a new frame has started based on packet sequence number.
  /// The packet sequence resets to 0 at the start of each frame.
  /// @param current_pkt_seq The current packet's sequence number
  /// @return true if a new frame has started
  bool check_scan_completed(uint16_t current_pkt_seq)
  {
    constexpr uint16_t safe_range = 10;

    bool completed = false;

    if (current_pkt_seq > max_pkt_seq_) {
      max_pkt_seq_ = current_pkt_seq;
    }

    if (current_pkt_seq < safe_seq_min_) {
      prev_pkt_seq_ = current_pkt_seq;
      completed = true;
      seq_looped_ = true;
    } else if (current_pkt_seq < prev_pkt_seq_) {
      // Ignore out-of-window reversals to match the vendor split strategy.
    } else if (current_pkt_seq <= safe_seq_max_) {
      prev_pkt_seq_ = current_pkt_seq;
    } else {
      if (prev_pkt_seq_ == 0) {
        prev_pkt_seq_ = current_pkt_seq;
      }
    }

    safe_seq_min_ = (prev_pkt_seq_ > safe_range) ? (prev_pkt_seq_ - safe_range) : 0;
    safe_seq_max_ = prev_pkt_seq_ + safe_range;

    return completed;
  }

  /// @brief Decode all points from a single packet
  void decode_packet()
  {
    uint64_t packet_timestamp_ns = robosense_packet::get_timestamp_ns(packet_);
    bool is_dual_return = (packet_.header.return_mode.value() == DirectionalReturnModeFlags::dual);

    for (size_t blk = 0; blk < SensorT::packet_t::n_blocks; ++blk) {
      auto & block = packet_.body.blocks[blk];

      for (size_t chan = 0; chan < SensorT::packet_t::n_channels; ++chan) {
        auto & unit = block.units[chan];

        const uint64_t packet_to_scan_offset_ns = packet_timestamp_ns - decode_scan_timestamp_ns_;
        const int point_relative_time_offset_ns = sensor_.get_packet_relative_point_time_offset(
          packet_, static_cast<uint32_t>(blk), static_cast<uint32_t>(chan), sensor_configuration_);
        const uint32_t point_time_stamp_ns = static_cast<uint32_t>(
          static_cast<int64_t>(packet_to_scan_offset_ns) + point_relative_time_offset_ns);

        // First return (always present)
        float dist = unit.distance.value() * distance_resolution_m_;
        if (dist >= SensorT::min_range && dist <= SensorT::max_range && dist != 0.0f) {
          NebulaPoint point;
          point.distance = dist;
          point.intensity = unit.reflectivity.value();
          point.return_type = is_dual_return ? static_cast<uint8_t>(ReturnType::STRONGEST)
                                             : static_cast<uint8_t>(ReturnType::STRONGEST);
          point.channel = static_cast<uint16_t>(chan);
          point.time_stamp = point_time_stamp_ns;
          if constexpr (SensorT::has_custom_projection) {
            sensor_.populate_point_xyz(
              point, packet_, static_cast<uint32_t>(blk), static_cast<uint32_t>(chan),
              *angle_corrector_);
          } else {
            populate_point_xyz(point, unit, dist);
          }
          decode_pc_->emplace_back(point);
        }

        // Second return (only for sensors with radius_sd, e.g. EMX, in dual mode)
        if (is_dual_return) {
          decode_second_return(unit, chan, point_time_stamp_ns, static_cast<uint32_t>(blk));
        }
      }
    }
  }

  /// @brief Decode second return from units that have radius_sd/intensity_sd (e.g. EMX).
  /// SFINAE-enabled: only compiles for unit types with radius_sd and intensity_sd members.
  template <typename U>
  auto decode_second_return(const U & unit, size_t chan, uint32_t time_stamp_ns, uint32_t block_id)
    -> decltype(unit.radius_sd.value(), unit.intensity_sd.value(), void())
  {
    float dist_sd = unit.radius_sd.value() * distance_resolution_m_;
    if (dist_sd < SensorT::min_range || dist_sd > SensorT::max_range || dist_sd == 0.0f) {
      return;
    }

    NebulaPoint point;
    point.distance = dist_sd;
    point.intensity = unit.intensity_sd.value();
    point.return_type = static_cast<uint8_t>(ReturnType::SECONDSTRONGEST);
    point.channel = static_cast<uint16_t>(chan);
    point.time_stamp = time_stamp_ns;
    if constexpr (SensorT::has_custom_projection) {
      sensor_.populate_point_xyz(
        point, packet_, block_id, static_cast<uint32_t>(chan), *angle_corrector_);
    } else {
      populate_point_xyz(point, unit, dist_sd);
    }
    decode_pc_->emplace_back(point);
  }

  /// @brief Fallback for units without second return data (E1, EM4)
  void decode_second_return(...) {}

  /// @brief Populate XYZ coordinates from direction vectors (for sensors with x, y, z in Unit).
  /// Uses SFINAE to detect members.
  template <typename U>
  auto populate_point_xyz(NebulaPoint & point, const U & unit, float dist)
    -> decltype(unit.x.value(), unit.y.value(), unit.z.value(), void())
  {
    float dir_x = static_cast<float>(unit.x.value()) / vector_base_;
    float dir_y = static_cast<float>(unit.y.value()) / vector_base_;
    float dir_z = static_cast<float>(unit.z.value()) / vector_base_;

    point.x = dir_x * dist;
    point.y = dir_y * dist;
    point.z = dir_z * dist;

    float xy_dist = std::sqrt(point.x * point.x + point.y * point.y);
    point.azimuth = std::atan2(point.y, point.x);
    point.elevation = std::atan2(point.z, xy_dist);
  }

  /// @brief Fallback for sensors without direction vectors (e.g. EM4)
  void populate_point_xyz(NebulaPoint & point, ...)
  {
    // XYZ will be computed from DIFOP2 calibration in a future step.
    point.x = 0.0f;
    point.y = 0.0f;
    point.z = 0.0f;
    point.azimuth = 0.0f;
    point.elevation = 0.0f;
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration (unused for directional sensors, kept for API)
  explicit RobosenseDecoderDirectional(
    const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & calibration_configuration)
  : sensor_configuration_(sensor_configuration),
    logger_(rclcpp::get_logger("RobosenseDecoderDirectional"))
  {
    angle_corrector_ =
      AngleCorrectorFactory<typename SensorT::angle_corrector_t, SensorT::has_custom_projection>::
        create(calibration_configuration);

    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, sensor_configuration_);

    decode_pc_.reset(new NebulaPointCloud);
    output_pc_.reset(new NebulaPointCloud);

    decode_pc_->reserve(SensorT::max_scan_buffer_points);
    output_pc_->reserve(SensorT::max_scan_buffer_points);
  }

  int unpack(const std::vector<uint8_t> & msop_packet) override
  {
    if (!parse_packet(msop_packet)) {
      return -1;
    }

    uint64_t packet_timestamp_ns = robosense_packet::get_timestamp_ns(packet_);

    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = packet_timestamp_ns;
    }

    if (has_scanned_) {
      has_scanned_ = false;
    }

    uint16_t pkt_seq = packet_.header.pkt_seq.value();

    bool scan_completed = check_scan_completed(pkt_seq);
    if (scan_completed) {
      std::swap(decode_pc_, output_pc_);
      decode_pc_->clear();
      has_scanned_ = true;
      output_scan_timestamp_ns_ = decode_scan_timestamp_ns_;
      decode_scan_timestamp_ns_ = packet_timestamp_ns;
    }

    decode_packet();

    return static_cast<int>(pkt_seq);
  }

  bool has_scanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace nebula::drivers
