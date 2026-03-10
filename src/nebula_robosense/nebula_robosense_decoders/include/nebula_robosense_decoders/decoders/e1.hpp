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

#include "nebula_robosense_decoders/decoders/robosense_packet.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor_directional.hpp"

#include "boost/endian/buffers.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
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
};

#pragma pack(pop)
}  // namespace robosense_packet::e1

class E1
: public RobosenseSensorDirectional<robosense_packet::e1::Packet, robosense_packet::e1::InfoPacket>
{
private:
  static constexpr uint8_t sync_mode_gps_flag = 0x00;
  static constexpr uint8_t sync_mode_e2e_flag = 0x02;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;

public:
  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 260000;

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
};
}  // namespace nebula::drivers
