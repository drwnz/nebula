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

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_robosense_decoders/decoders/angle_corrector.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace nebula::drivers
{

/// @brief Shared return mode flag values used by directional Robosense sensors (E1, EM4, EMX)
struct DirectionalReturnModeFlags
{
  static constexpr uint8_t dual = 0x00;
  static constexpr uint8_t strongest = 0x04;
  static constexpr uint8_t last = 0x05;
  static constexpr uint8_t nearest = 0x06;
};

/// @brief Shared sync status flag values used by directional Robosense sensors (E1, EM4, EMX)
struct DirectionalSyncStatusFlags
{
  static constexpr uint8_t failed = 0x00;
  static constexpr uint8_t success = 0x01;
  static constexpr uint8_t timeout = 0x02;
};

/// @brief Intermediate base class for directional Robosense sensors (E1, EM4, EMX).
/// Provides shared default implementations for methods that are identical across these sensors.
/// @tparam PacketT The MSOP packet type
/// @tparam InfoPacketT The DIFOP info packet type
template <typename PacketT, typename InfoPacketT>
class RobosenseSensorDirectional : public RobosenseSensor<PacketT, InfoPacketT>
{
public:
  int get_packet_relative_point_time_offset(
    const PacketT & /*packet*/, const uint32_t /*block_id*/, const uint32_t /*channel_id*/,
    const std::shared_ptr<const RobosenseSensorConfiguration> & /*sensor_configuration*/) override
  {
    // Directional sensors embed time offsets per-block. The decoder handles this directly.
    return 0;
  }

  ReturnMode get_return_mode(const InfoPacketT & /*info_packet*/) override
  {
    // Directional sensors do not expose return mode through the DIFOP packet in the same way.
    return ReturnMode::UNKNOWN;
  }

  RobosenseCalibrationConfiguration get_sensor_calibration(
    const InfoPacketT & /*info_packet*/) override
  {
    // Directional sensors use DIFOP2 calibration or embedded direction vectors.
    // No standard azimuth/elevation calibration table.
    return RobosenseCalibrationConfiguration();
  }

  bool get_sync_status(const InfoPacketT & /*info_packet*/) override { return false; }

  /// @brief Helper to populate common sync status info fields
  /// @param sensor_info The map to populate
  /// @param sync_status_value The raw sync status byte
  static void populate_sync_status_info(
    std::map<std::string, std::string> & sensor_info, uint8_t sync_status_value)
  {
    switch (sync_status_value) {
      case DirectionalSyncStatusFlags::failed:
        sensor_info["sync_status"] = "failed";
        break;
      case DirectionalSyncStatusFlags::success:
        sensor_info["sync_status"] = "success";
        break;
      case DirectionalSyncStatusFlags::timeout:
        sensor_info["sync_status"] = "timeout";
        break;
      default:
        sensor_info["sync_status"] = "n/a";
    }
  }

  static ReturnMode return_mode_from_wave_mode(uint8_t wave_mode)
  {
    switch (wave_mode) {
      case DirectionalReturnModeFlags::dual:
        return ReturnMode::DUAL;
      case DirectionalReturnModeFlags::strongest:
        return ReturnMode::SINGLE_STRONGEST;
      case DirectionalReturnModeFlags::last:
        return ReturnMode::SINGLE_LAST;
      case DirectionalReturnModeFlags::nearest:
        return ReturnMode::SINGLE_FIRST;
      default:
        return ReturnMode::UNKNOWN;
    }
  }

  static void populate_point_from_corrected_angle(
    NebulaPoint & point, const CorrectedAngleData & corrected_data, uint16_t channel)
  {
    const float xy_dist = point.distance * corrected_data.cos_elevation;
    point.x = xy_dist * corrected_data.cos_azimuth;
    point.y = xy_dist * corrected_data.sin_azimuth;
    point.z = point.distance * corrected_data.sin_elevation;
    point.azimuth = corrected_data.azimuth_rad;
    point.elevation = corrected_data.elevation_rad;
    point.channel = channel;
  }
};

}  // namespace nebula::drivers
