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

#include <nebula_core_runtime/packet_router.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace nebula::drivers
{

void PacketRouter::configure(const std::vector<PacketChannelRequirement> & requirements)
{
  udp_port_map_.clear();
  can_id_map_.clear();
  for (const auto & req : requirements) {
    if (req.transport == SensorTransportKind::UDP && req.udp_destination_port.has_value()) {
      udp_port_map_.emplace(*req.udp_destination_port, req);
    } else if (req.transport == SensorTransportKind::CAN && req.can_id.has_value()) {
      can_id_map_.emplace(*req.can_id, req);
    }
  }
}

bool PacketRouter::route(SensorPacket & packet)
{
  metrics_.processed_packets++;

  bool matched = false;
  if ((packet.transport == SensorTransportKind::UDP || packet.transport == SensorTransportKind::Replay) && packet.destination.has_value()) {
    auto range = udp_port_map_.equal_range(packet.destination->port);
    for (auto it = range.first; it != range.second; ++it) {
        if (!it->second.payload_signature.has_value() || match_signature(packet.payload, *it->second.payload_signature)) {
            packet.channel = it->second.channel;
            matched = true;
            break;
        }
    }
  } else if (packet.transport == SensorTransportKind::CAN && packet.can.has_value()) {
    auto range = can_id_map_.equal_range(packet.can->can_id);
    for (auto it = range.first; it != range.second; ++it) {
        if (!it->second.payload_signature.has_value() || match_signature(packet.payload, *it->second.payload_signature)) {
            packet.channel = it->second.channel;
            matched = true;
            break;
        }
    }
  }

  if (matched) {
      metrics_.matched_packets++;
      return true;
  }

  metrics_.dropped_packets++;
  return false;
}

const SensorProgress & PacketRouter::get_metrics() const
{
  return metrics_;
}

bool PacketRouter::match_signature(const std::vector<uint8_t> & payload, const std::string & signature)
{
    if (signature.empty()) return true;
    
    // Simple hex signature matching for now
    std::vector<uint8_t> sig_bytes;
    for (size_t i = 0; i < signature.length(); i += 2) {
        std::string byteString = signature.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(strtol(byteString.c_str(), NULL, 16));
        sig_bytes.push_back(byte);
    }

    if (payload.size() < sig_bytes.size()) return false;
    
    return std::equal(sig_bytes.begin(), sig_bytes.end(), payload.begin());
}

}  // namespace nebula::drivers
