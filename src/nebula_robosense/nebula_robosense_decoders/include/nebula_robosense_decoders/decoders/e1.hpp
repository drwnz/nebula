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
  big_uint64_buf_t header;             // offset 0, 8 bytes  (DIFOP header)
  uint8_t res0[8];                     // offset 8, 8 bytes  (Reserved)
  uint8_t sw_version[3];               // offset 16, 3 bytes (SW Version)
  uint8_t res1[1];                     // offset 19, 1 byte  (Reserved)
  uint8_t sn[6];                       // offset 20, 6 bytes (Serial Number)
  uint8_t res2[18];                    // offset 26, 18 bytes (Reserved)
  IpAddress local_ip;                  // offset 44, 4 bytes (LiDAR IP source)
  IpAddress net_mask;                  // offset 48, 4 bytes (Subnet mask)
  MacAddress mac_address;              // offset 52, 6 bytes (MAC address)
  IpAddress msop_remote_ip;            // offset 58, 4 bytes (MSOP remote IP)
  big_uint16_buf_t msop_local_port;    // offset 62, 2 bytes (MSOP local port)
  big_uint16_buf_t msop_remote_port;   // offset 64, 2 bytes (MSOP remote port)
  IpAddress difop_remote_ip;           // offset 66, 4 bytes (DIFOP remote IP)
  big_uint16_buf_t difop_local_port;   // offset 70, 2 bytes (DIFOP local port)
  big_uint16_buf_t difop_remote_port;  // offset 72, 2 bytes (DIFOP remote port)
  uint8_t res3[25];                    // offset 74, 25 bytes (Reserved)
  big_uint8_buf_t frequency_setting;   // offset 99, 1 byte  (Frame rate)
  big_uint8_buf_t return_mode;         // offset 100, 1 byte (Return mode)
  big_uint8_buf_t time_mode;           // offset 101, 1 byte (Time sync mode)
  big_uint8_buf_t time_sync_status;    // offset 102, 1 byte (Time sync status)
  Timestamp time;                      // offset 103, 10 bytes (Timestamp)
  big_uint8_buf_t phy_mode;            // offset 113, 1 byte (PHY mode)
  uint8_t res4[94];                    // offset 114, 94 bytes (Reserved)
  big_int32_buf_t acceIx;              // offset 208, 4 bytes (IMU accel X)
  big_int32_buf_t acceIy;              // offset 212, 4 bytes (IMU accel Y)
  big_int32_buf_t acceIz;              // offset 216, 4 bytes (IMU accel Z)
  big_int32_buf_t gyrox;               // offset 220, 4 bytes (IMU gyro X)
  big_int32_buf_t gyroy;               // offset 224, 4 bytes (IMU gyro Y)
  big_int32_buf_t gyroz;               // offset 228, 4 bytes (IMU gyro Z)
  uint8_t res5[24];                    // offset 232, 24 bytes (Reserved)
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

  // E1 DIFOP return mode byte values (from user manual)
  static constexpr uint8_t return_mode_farthest = 0x00;
  static constexpr uint8_t return_mode_strongest = 0x04;
  static constexpr uint8_t return_mode_nearest = 0x07;
  static constexpr uint8_t return_mode_2nd_strongest = 0x08;
  static constexpr uint8_t return_mode_strongest_farthest = 0x09;
  static constexpr uint8_t return_mode_nearest_farthest = 0x0A;
  static constexpr uint8_t return_mode_strongest_2nd = 0x0B;

  static constexpr int VECTOR_BASE = 32768;

public:
  static constexpr bool has_custom_projection = false;
  typedef AngleCorrector angle_corrector_t;

  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 260000;

  ReturnMode get_return_mode(const robosense_packet::e1::InfoPacket & info_packet) override
  {
    switch (info_packet.return_mode.value()) {
      case return_mode_farthest:
        return ReturnMode::SINGLE_LAST;
      case return_mode_strongest:
        return ReturnMode::SINGLE_STRONGEST;
      case return_mode_nearest:
        return ReturnMode::SINGLE_FIRST;
      case return_mode_2nd_strongest:
        return ReturnMode::SINGLE_STRONGEST;
      case return_mode_strongest_farthest:
      case return_mode_nearest_farthest:
      case return_mode_strongest_2nd:
        return ReturnMode::DUAL;
      default:
        return ReturnMode::UNKNOWN;
    }
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

    // Network config from DIFOP
    sensor_info["sensor_ip"] = info_packet.local_ip.to_string();
    sensor_info["dest_ip"] = info_packet.msop_remote_ip.to_string();
    sensor_info["msop_dst_port"] = std::to_string(info_packet.msop_remote_port.value());
    sensor_info["difop_dst_port"] = std::to_string(info_packet.difop_remote_port.value());
    return sensor_info;
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
