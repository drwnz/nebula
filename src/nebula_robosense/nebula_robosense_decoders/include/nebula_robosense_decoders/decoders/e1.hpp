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

#include "nebula_robosense_decoders/decoders/angle_corrector.hpp"
#include "nebula_robosense_decoders/decoders/robosense_packet.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor_directional.hpp"

#include "boost/endian/buffers.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

using namespace boost::endian;  // NOLINT(build/namespaces)

namespace nebula::drivers
{
namespace robosense_packet::e1
{
#pragma pack(push, 1)

struct Header
{
  big_uint32_buf_t header_id;
  big_uint16_buf_t pkt_seq;
  big_uint16_buf_t protocol_version;
  big_uint8_buf_t return_mode;
  big_uint8_buf_t time_mode;
  Timestamp timestamp;
  big_uint8_buf_t frame_sync;
  uint8_t reserved_first[9];
  big_uint8_buf_t lidar_type;
  big_uint8_buf_t temperature;
};

struct Unit
{
  big_uint16_buf_t distance;
  big_int16_buf_t x;
  big_int16_buf_t y;
  big_int16_buf_t z;
  big_uint8_buf_t reflectivity;
  big_uint8_buf_t point_attribute;
};

struct Block
{
  typedef Unit unit_t;
  big_uint16_buf_t time_offset;
  Unit units[1];
};

struct Body
{
  typedef Block block_t;
  Block blocks[96];
};

struct Packet : public PacketBase<1, 96, 1, 1>
{
  typedef Body body_t;
  Header header;
  body_t body;
  uint8_t tail[16];
};

struct InfoPacket
{
  big_uint64_buf_t header;
  uint8_t reserved1[93];
  big_uint8_buf_t time_mode;
  big_uint8_buf_t time_sync_status;
  Timestamp time;
  uint8_t reserved2[95];
  big_int32_buf_t acceIx;
  big_int32_buf_t acceIy;
  big_int32_buf_t acceIz;
  big_int32_buf_t gyrox;
  big_int32_buf_t gyroy;
  big_int32_buf_t gyroz;
  uint8_t reserved3[24];
};  // total: 256 bytes

#pragma pack(pop)
}  // namespace robosense_packet::e1
}  // namespace nebula::drivers

namespace nebula::drivers::robosense_packet
{
/// @brief Specialization for E1 as it doesn't have range_resolution in header
template <>
inline double get_dis_unit<e1::Packet>(const e1::Packet &)
{
  return 0.005;
}
}  // namespace nebula::drivers::robosense_packet

namespace nebula::drivers
{
class E1
: public RobosenseSensorDirectional<robosense_packet::e1::Packet, robosense_packet::e1::InfoPacket>
{
private:
  static constexpr uint8_t sync_mode_gps_flag = 0x00;
  static constexpr uint8_t sync_mode_e2e_flag = 0x02;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;

  static constexpr int VECTOR_BASE = 32768;

public:
  static constexpr bool has_custom_projection = false;
  typedef AngleCorrector angle_corrector_t;

  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 260000;

  ReturnMode get_return_mode(const robosense_packet::e1::InfoPacket & /*info_packet*/) override
  {
    return ReturnMode::UNKNOWN;
  }

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::e1::InfoPacket & info_packet) override
  {
    std::map<std::string, std::string> sensor_info;

    switch (info_packet.time_mode.value()) {
      case sync_mode_gps_flag:
        sensor_info["time_sync_mode"] = "internal";
        break;
      case sync_mode_e2e_flag:
        sensor_info["time_sync_mode"] = "e2e";
        break;
      case sync_mode_gptp_flag:
        sensor_info["time_sync_mode"] = "gptp";
        break;
      default:
        sensor_info["time_sync_mode"] = "n/a";
        break;
    }

    populate_sync_status_info(sensor_info, info_packet.time_sync_status.value());
    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());

    return sensor_info;
  }

  bool get_sync_status(const robosense_packet::e1::InfoPacket & info_packet) override
  {
    return info_packet.time_sync_status.value() == DirectionalSyncStatusFlags::success;
  }

  int get_packet_relative_point_time_offset(
    const robosense_packet::e1::Packet & packet, const uint32_t block_id,
    const uint32_t /*channel_id*/,
    const std::shared_ptr<const RobosenseSensorConfiguration> & /*sensor_configuration*/) override
  {
    return static_cast<int>(packet.body.blocks[block_id].time_offset.value()) * 1000;
  }

  template <typename CorrectorT>
  void populate_point_xyz(
    ::nebula::drivers::NebulaPoint & point, const robosense_packet::e1::Packet & pkt,
    uint32_t block_id, uint32_t channel_id, CorrectorT & /*angle_corrector*/)
  {
    const auto & unit = pkt.body.blocks[block_id].units[channel_id];

    // E1 provides x, y, z direction vectors
    int16_t vx = unit.x.value();
    int16_t vy = unit.y.value();
    int16_t vz = unit.z.value();

    // distance is already in point.distance (from get_distance call in decoder)
    // Formula: x = vx * distance / VECTOR_BASE
    point.x = static_cast<float>(vx) * point.distance / static_cast<float>(VECTOR_BASE);
    point.y = static_cast<float>(vy) * point.distance / static_cast<float>(VECTOR_BASE);
    point.z = static_cast<float>(vz) * point.distance / static_cast<float>(VECTOR_BASE);

    // azimuth/elevation can be derived if needed, but NebulaPoint mainly uses x,y,z
    point.azimuth = atan2f(point.y, point.x);
    point.elevation = asinf(point.z / point.distance);
    point.channel = static_cast<uint16_t>(channel_id);
  }
};
}  // namespace nebula::drivers
