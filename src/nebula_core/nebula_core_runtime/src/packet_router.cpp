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

namespace nebula::drivers
{

void PacketRouter::configure(const std::vector<PacketChannelRequirement> & requirements)
{
  udp_port_map_.clear();
  can_id_map_.clear();
  for (const auto & req : requirements) {
    if (req.transport == SensorTransportKind::UDP && req.udp_destination_port.has_value()) {
      udp_port_map_[*req.udp_destination_port] = req;
    } else if (req.transport == SensorTransportKind::CAN && req.can_id.has_value()) {
      can_id_map_[*req.can_id] = req;
    }
  }
}

bool PacketRouter::route(SensorPacket & packet)
{
  metrics_.processed_packets++;

  if ((packet.transport == SensorTransportKind::UDP || packet.transport == SensorTransportKind::Replay) && packet.destination.has_value()) {
    auto it = udp_port_map_.find(packet.destination->port);
    if (it != udp_port_map_.end()) {
      packet.channel = it->second.channel;
      metrics_.matched_packets++;
      return true;
    }
  } else if (packet.transport == SensorTransportKind::CAN && packet.can.has_value()) {
    auto it = can_id_map_.find(packet.can->can_id);
    if (it != can_id_map_.end()) {
      packet.channel = it->second.channel;
      metrics_.matched_packets++;
      return true;
    }
  }

  metrics_.dropped_packets++;
  return false;
}

const SensorProgress & PacketRouter::get_metrics() const
{
  return metrics_;
}

}  // namespace nebula::drivers
