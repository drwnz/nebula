// Copyright 2026 TIER IV, Inc.

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_robosense_common/robosense_common.hpp"
#include "nebula_robosense_decoders/decoders/e1.hpp"
#include "nebula_robosense_decoders/robosense_driver.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

namespace
{

using nebula::drivers::RobosenseCalibrationConfiguration;
using nebula::drivers::RobosenseDriver;
using nebula::drivers::RobosenseSensorConfiguration;
using nebula::drivers::SensorModel;
using Packet = nebula::drivers::robosense_packet::e1::Packet;

std::vector<uint8_t> serialize_packet(const Packet & packet)
{
  std::vector<uint8_t> bytes(sizeof(Packet));
  std::memcpy(bytes.data(), &packet, sizeof(Packet));
  return bytes;
}

Packet make_packet(uint16_t pkt_seq)
{
  Packet packet{};
  packet.header.header_id = 0x55AA5AA5;
  packet.header.pkt_seq = pkt_seq;
  packet.header.protocol_version = 0x0001;
  packet.header.return_mode = 0x04;
  packet.header.time_mode = 0x00;
  packet.header.timestamp.seconds = 1;
  packet.header.timestamp.microseconds = 0;
  packet.header.temperature = 80;

  for (size_t block_idx = 0; block_idx < Packet::n_blocks; ++block_idx) {
    auto & block = packet.body.blocks[block_idx];
    block.time_offset = static_cast<uint16_t>(block_idx);
    block.units[0].distance = 1000;
    block.units[0].reflectivity = static_cast<uint8_t>(block_idx);
    block.units[0].x = static_cast<int16_t>(1000 + static_cast<int>(block_idx));
    block.units[0].y = static_cast<int16_t>(2000 + static_cast<int>(block_idx));
    block.units[0].z = static_cast<int16_t>(3000 + static_cast<int>(block_idx));
  }

  return packet;
}

std::shared_ptr<const RobosenseSensorConfiguration> make_sensor_config()
{
  auto config = std::make_shared<RobosenseSensorConfiguration>();
  config->sensor_model = SensorModel::ROBOSENSE_E1;
  config->return_mode = nebula::drivers::ReturnMode::SINGLE_STRONGEST;
  config->frame_id = "robosense";
  return config;
}

std::shared_ptr<const RobosenseCalibrationConfiguration> make_calibration_config()
{
  return std::make_shared<RobosenseCalibrationConfiguration>();
}

}  // namespace

TEST(RobosenseE1DecoderTest, PacketLayoutMetadataMatchesVendorFormat)
{
  EXPECT_EQ(Packet::n_blocks, 96U);
  EXPECT_EQ(Packet::n_channels, 1U);
}

TEST(RobosenseE1DecoderTest, DecodesSuccessiveBlocksAsDistinctProjectedPoints)
{
  RobosenseDriver driver(make_sensor_config(), make_calibration_config());

  auto first_packet = serialize_packet(make_packet(1));
  auto second_packet = serialize_packet(make_packet(0));

  auto first_result = driver.parse_cloud_packet(first_packet);
  EXPECT_EQ(std::get<0>(first_result), nullptr);

  auto second_result = driver.parse_cloud_packet(second_packet);
  auto pointcloud = std::get<0>(second_result);

  ASSERT_NE(pointcloud, nullptr);
  ASSERT_EQ(pointcloud->size(), Packet::n_blocks);

  constexpr float kDistanceMeters = 1000.0f * 0.005f;
  constexpr float kVectorBase = 32768.0f;

  const auto & first_point = pointcloud->at(0);
  EXPECT_NEAR(first_point.x, 1000.0f * kDistanceMeters / kVectorBase, 1e-5f);
  EXPECT_NEAR(first_point.y, 2000.0f * kDistanceMeters / kVectorBase, 1e-5f);
  EXPECT_NEAR(first_point.z, 3000.0f * kDistanceMeters / kVectorBase, 1e-5f);
  EXPECT_EQ(first_point.intensity, 0U);
  EXPECT_EQ(first_point.channel, 0U);

  const auto & second_point = pointcloud->at(1);
  EXPECT_NEAR(second_point.x, 1001.0f * kDistanceMeters / kVectorBase, 1e-5f);
  EXPECT_NEAR(second_point.y, 2001.0f * kDistanceMeters / kVectorBase, 1e-5f);
  EXPECT_NEAR(second_point.z, 3001.0f * kDistanceMeters / kVectorBase, 1e-5f);
  EXPECT_EQ(second_point.intensity, 1U);
  EXPECT_EQ(second_point.channel, 0U);
}
