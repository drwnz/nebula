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

#include <nebula_core_common/sensor_output.hpp>
#include <nebula_core_common/sensor_packet.hpp>
#include <nebula_core_common/sensor_runtime_common.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace nebula::drivers;

TEST(TestCoreTypes, SensorPacket)
{
  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.channel = SensorPacketChannel::Data;
  packet.timestamp_ns = 123456789;
  packet.payload = {0x01, 0x02, 0x03};

  EXPECT_EQ(packet.transport, SensorTransportKind::UDP);
  EXPECT_EQ(packet.channel, SensorPacketChannel::Data);
  EXPECT_EQ(packet.timestamp_ns, 123456789);
  EXPECT_EQ(packet.payload.size(), 3u);
}

TEST(TestCoreTypes, SensorDecodedOutput)
{
  SensorDecodedOutput output;
  output.kind = SensorOutputKind::PointCloud;
  output.timestamp_ns = 987654321;
  output.sensor_id = "test_sensor";
  output.payload = std::string("test_payload");

  EXPECT_EQ(output.kind, SensorOutputKind::PointCloud);
  EXPECT_EQ(output.timestamp_ns, 987654321);
  EXPECT_EQ(output.sensor_id, "test_sensor");
  EXPECT_TRUE(output.payload.has_value());
  EXPECT_EQ(std::any_cast<std::string>(output.payload), "test_payload");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
