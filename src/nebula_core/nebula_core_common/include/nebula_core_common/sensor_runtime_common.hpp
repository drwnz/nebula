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

// Simplified config that wraps the existing ones or provides a generic way to pass parameters
struct SensorConfiguration
{
  SensorModel sensor_model;
  std::string frame_id;
  std::string sensor_ip;
  std::string host_ip;
  uint16_t data_port;
  // This can be extended to include all fields from LidarConfigurationBase, etc.
  // For now, let's keep it minimal or decide how to unify.
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_RUNTIME_COMMON_HPP
