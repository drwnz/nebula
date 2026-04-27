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

#ifndef NEBULA_SENSOR_REQUIREMENTS_HPP
#define NEBULA_SENSOR_REQUIREMENTS_HPP

#include <nebula_core_common/sensor_packet.hpp>

#include <optional>
#include <string>

namespace nebula::drivers
{
struct PacketChannelRequirement
{
  SensorTransportKind transport;
  SensorPacketChannel channel;
  bool required;
  std::optional<uint16_t> udp_destination_port;
  std::optional<uint32_t> can_id;
  std::optional<std::string> payload_signature;
};

struct LiveTransportRequirement
{
  SensorTransportKind transport;
  SensorPacketChannel channel;
  bool required;
  std::string name;
  std::optional<uint16_t> port;
  std::optional<std::string> http_path;
  std::optional<uint32_t> can_id;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_REQUIREMENTS_HPP
