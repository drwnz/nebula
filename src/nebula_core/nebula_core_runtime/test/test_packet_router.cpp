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

#include <gtest/gtest.h>

namespace nebula::drivers::test
{

TEST(TestPacketRouter, RouteUdpPacket)
{
  PacketRouter router;
  
  std::vector<PacketChannelRequirement> requirements;
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.udp_destination_port = 2368;
  requirements.push_back(req);

  router.configure(requirements);

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  SensorEndpoint dst;
  dst.port = 2368;
  packet.destination = dst;
  
  EXPECT_TRUE(router.route(packet));
  EXPECT_EQ(packet.channel, SensorPacketChannel::Data);
  
  EXPECT_EQ(router.get_metrics().matched_packets, 1u);
  EXPECT_EQ(router.get_metrics().processed_packets, 1u);
}

TEST(TestPacketRouter, DropUnmatchedPacket)
{
  PacketRouter router;
  router.configure({});

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  SensorEndpoint dst;
  dst.port = 2368;
  packet.destination = dst;

  EXPECT_FALSE(router.route(packet));
  EXPECT_EQ(router.get_metrics().dropped_packets, 1u);
}

}  // namespace nebula::drivers::test
