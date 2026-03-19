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

#include "nebula_robosense_common/robosense_common.hpp"
#include "nebula_robosense_decoders/decoders/angle_corrector.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Angle corrector for EM4 sensors using Method 4 calculation
class AngleCorrectorEM4 : public AngleCorrector
{
private:
  std::vector<float> half_vcsel_yaw_offset_rad_;
  std::vector<float> pixel_pitch_rad_;
  std::vector<float> surface_pitch_offset_rad_;

public:
  explicit AngleCorrectorEM4(
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & sensor_calibration)
  : AngleCorrector(sensor_calibration)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error("Cannot instantiate AngleCorrectorEM4 without calibration data");
    }

    half_vcsel_yaw_offset_rad_.reserve(sensor_calibration->half_vcsel_yaw_offset.size());
    for (auto offset : sensor_calibration->half_vcsel_yaw_offset) {
      half_vcsel_yaw_offset_rad_.push_back(deg2rad(offset * 0.01f));
    }

    pixel_pitch_rad_.reserve(sensor_calibration->pixel_pitch.size());
    for (auto pitch : sensor_calibration->pixel_pitch) {
      pixel_pitch_rad_.push_back(deg2rad(pitch * 0.01f));
    }

    surface_pitch_offset_rad_.reserve(sensor_calibration->surface_pitch_offset.size());
    for (auto offset : sensor_calibration->surface_pitch_offset) {
      surface_pitch_offset_rad_.push_back(deg2rad(offset * 0.01f));
    }
  }

  /// @brief EM4-specific angle correction math (Method 4)
  /// @param raw_yaw The yaw_angle from MSOP header (0.01 degree)
  /// @param channel_id The channel index (0-259 or 0-519 depending on packet)
  /// @param mirror_id The mirror surface index (0-3 for A, B, C, D)
  /// @return Corrected angle data
  CorrectedAngleData get_corrected_angle_data_em4(
    int16_t raw_yaw, uint32_t channel_id, uint8_t mirror_id)
  {
    // yaw(j, i) = yaw_angle + HalfVcselYawOffset([i/20]+1)
    // Note: manual says [i/20]+1 for 1-based indexing, we use 0-based.
    size_t yaw_offset_idx = channel_id / 20;
    float yaw_rad = deg2rad(raw_yaw * 0.01f) + (yaw_offset_idx < half_vcsel_yaw_offset_rad_.size()
                                                  ? half_vcsel_yaw_offset_rad_[yaw_offset_idx]
                                                  : 0.f);

    // Pitch(j, i) = PixelPitch(i) + SurfacePitchOffset(j)
    float pitch_rad =
      (channel_id < pixel_pitch_rad_.size() ? pixel_pitch_rad_[channel_id] : 0.f) +
      (mirror_id < surface_pitch_offset_rad_.size() ? surface_pitch_offset_rad_[mirror_id] : 0.f);

    return {
      yaw_rad,
      pitch_rad,
      sinf(yaw_rad),
      cosf(yaw_rad),
      sinf(pitch_rad),
      cosf(pitch_rad),
      static_cast<uint16_t>(channel_id)  // TODO(drwnz): Verify if channel indexing needs remapping
    };
  }

  // Not used for EM4 in the standard way, as EM4 doesn't have a fixed azimuth grid
  CorrectedAngleData get_corrected_angle_data(uint32_t, uint32_t) override { return {}; }

  bool has_scanned(int current_azimuth, int last_azimuth) override
  {
    // EM4 uses packet sequence for frame boundaries, but azimuth can also be used.
    return current_azimuth < last_azimuth;
  }
};

}  // namespace nebula::drivers
