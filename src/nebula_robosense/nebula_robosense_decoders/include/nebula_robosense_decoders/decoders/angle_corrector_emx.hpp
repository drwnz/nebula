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
#include "nebula_robosense_decoders/decoders/angle_corrector.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Angle corrector for EMX sensors using polar calculations
class AngleCorrectorEMX : public AngleCorrector
{
private:
  std::vector<float> half_vcsel_yaw_offset_rad_;
  std::vector<float> pixel_pitch_rad_;
  std::vector<float> surface_pitch_offset_rad_;

public:
  explicit AngleCorrectorEMX(
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & sensor_calibration)
  : AngleCorrector(sensor_calibration)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error("Cannot instantiate AngleCorrectorEMX without calibration data");
    }

    half_vcsel_yaw_offset_rad_ =
      convert_angle_offsets_to_rad(sensor_calibration->half_vcsel_yaw_offset, 1.0f / 512.0f);

    pixel_pitch_rad_.reserve(192);
    bool use_default_pitch = true;
    for (auto pitch : sensor_calibration->pixel_pitch) {
      if (pitch != 0) {
        use_default_pitch = false;
        break;
      }
    }

    if (use_default_pitch) {
      // Match the vendor EMX fallback table:
      // START_ANGLE = -24.16 deg, STEP = 0.21 deg.
      // Our stored pitch values use the raw DIFOP units, so convert the vendor table accordingly.
      for (int i = 0; i < 192; ++i) {
        float pitch_deg = static_cast<float>(-4832 + i * 42) / 200.0f;
        pixel_pitch_rad_.push_back(deg2rad(pitch_deg));
      }
    } else {
      for (auto pitch : sensor_calibration->pixel_pitch) {
        // DIFOP units are 0.005 deg (raw/200 deg)
        pixel_pitch_rad_.push_back(deg2rad(static_cast<float>(pitch) / 200.0f));
      }
    }

    surface_pitch_offset_rad_ =
      convert_angle_offsets_to_rad(sensor_calibration->surface_pitch_offset, 1.0f / 200.0f);
  }

  /// @brief EMX-specific angle correction math
  /// @param raw_yaw The yaw_angle from MSOP header
  /// @param channel_id The channel index
  /// @param mirror_id The mirror surface index (0-1)
  /// @return Corrected angle data
  CorrectedAngleData get_corrected_angle_data_emx(
    int16_t raw_yaw, uint32_t channel_id, uint8_t mirror_id)
  {
    size_t yaw_offset_idx = channel_id / 8;
    float yaw_rad = deg2rad(static_cast<float>(raw_yaw) / 512.0f) +
                    (yaw_offset_idx < half_vcsel_yaw_offset_rad_.size()
                       ? half_vcsel_yaw_offset_rad_[yaw_offset_idx]
                       : 0.f);

    float pitch_rad =
      (channel_id < pixel_pitch_rad_.size() ? pixel_pitch_rad_[channel_id] : 0.f) +
      (mirror_id < surface_pitch_offset_rad_.size() ? surface_pitch_offset_rad_[mirror_id] : 0.f);

    return make_corrected_angle_data(yaw_rad, pitch_rad, static_cast<uint16_t>(channel_id));
  }

  CorrectedAngleData get_corrected_angle_data(uint32_t, uint32_t) override { return {}; }

  bool has_scanned(int current_azimuth, int last_azimuth) override
  {
    return current_azimuth < last_azimuth;
  }
};

}  // namespace nebula::drivers
