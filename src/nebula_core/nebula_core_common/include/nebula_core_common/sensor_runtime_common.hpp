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

#ifndef NEBULA_SENSOR_RUNTIME_COMMON_HPP
#define NEBULA_SENSOR_RUNTIME_COMMON_HPP

#include <nebula_core_common/nebula_common.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <map>

namespace nebula::drivers
{
struct SensorModelInfo
{
  SensorModel model;
  std::string name;
  std::string description;
};

struct SensorPluginMetadata
{
  std::string vendor;
  std::string package_name;
  std::string library_path;
  std::string factory_symbol;
  std::vector<SensorModel> supported_models;
};

struct SensorProgress
{
  uint64_t processed_packets = 0;
  uint64_t matched_packets = 0;
  uint64_t dropped_packets = 0;
  uint64_t decoded_packets = 0;
  uint64_t output_count = 0;
  uint64_t error_count = 0;
};

enum class SensorErrorType {
  None,
  ConfigError,
  TransportError,
  ProtocolError,
  DecoderError,
  InternalError,
};

struct SensorError
{
  SensorErrorType type;
  std::string message;
  uint64_t timestamp_ns;
};

enum class SensorPacketResult {
  Success,
  Buffered,
  Ignored,
  Error,
};

struct SensorConfiguration
{
  SensorModel sensor_model{SensorModel::UNKNOWN};
  std::string frame_id;
  std::string sensor_ip;
  std::string host_ip;
  uint16_t data_port{0};
  uint16_t gnss_port{0};
  
  std::string calibration_file;
  
  ReturnMode return_mode{ReturnMode::UNKNOWN};
  
  struct {
    struct {
      float start{0};
      float end{360};
    } azimuth;
    struct {
      float start{-90};
      float end{90};
    } elevation;
  } fov;

  double rotation_speed{600};
  double min_range{0.1};
  double max_range{200.0};
  
  // Generic key-value store for vendor-specific parameters
  std::map<std::string, std::string> extra_params;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_RUNTIME_COMMON_HPP
