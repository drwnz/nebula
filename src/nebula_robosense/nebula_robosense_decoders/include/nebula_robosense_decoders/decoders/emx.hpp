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

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_robosense_decoders/decoders/robosense_packet.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor_directional.hpp"

#include <boost/endian/buffers.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace boost::endian;  // NOLINT(build/namespaces)

namespace nebula::drivers
{
namespace robosense_packet::emx
{
#pragma pack(push, 1)

struct Header
{
  big_uint32_buf_t header_id;         // 0
  big_uint16_buf_t pkt_seq;           // 4
  big_uint16_buf_t protocol_version;  // 6
  big_uint8_buf_t return_mode;        // 8
  big_uint8_buf_t time_mode;          // 9
  Timestamp timestamp;                // 10
  uint8_t reserved[10];               // 20
  big_uint8_buf_t lidar_type;         // 30
  big_uint8_buf_t temperature;        // 31
};

struct Unit
{
  big_uint16_buf_t distance;        // 0 (radius_ft)
  big_uint16_buf_t radius_sd;       // 2
  big_int16_buf_t x;                // 4
  big_int16_buf_t y;                // 6
  big_int16_buf_t z;                // 8
  big_uint8_buf_t reflectivity;     // 10 (intensity_ft)
  big_uint8_buf_t intensity_sd;     // 11
  big_uint8_buf_t point_attribute;  // 12
};

struct Block
{
  typedef Unit unit_t;
  uint8_t time_offset;  // 0
  Unit units[2];        // 1 (Total 27 bytes per block)
};

struct Body
{
  typedef Block block_t;
  Block blocks[50];  // Total 1350 bytes
};

struct Footer
{
  uint8_t reserved[16];
  big_uint32_buf_t crc32;
  big_uint16_buf_t rolling_counter;
};

struct Packet : public robosense_packet::PacketBase<50, 2, 1, 1>
{
  typedef robosense_packet::Body<
    robosense_packet::Block<Unit, Packet::n_channels>, Packet::n_blocks>
    body_t;
  Header header;
  Body body;
  Footer footer;
};

struct InfoPacket
{
  big_uint32_buf_t status_hdr;             // offset 0, 4 bytes
  uint8_t reserved[20];                    // offset 4, 20 bytes
  uint8_t sw_version[3];                   // offset 24, 3 bytes
  uint8_t hw_version[2];                   // offset 27, 2 bytes
  uint8_t reserved1[6];                    // offset 29, 6 bytes
  uint8_t customer_sn[16];                 // offset 35, 16 bytes
  big_uint8_buf_t work_mode;               // offset 51, 1 byte
  big_uint8_buf_t frame_rate;              // offset 52, 1 byte
  big_uint8_buf_t wave_mode;               // offset 53, 1 byte  (return mode)
  big_uint8_buf_t roi_mode;                // offset 54, 1 byte
  big_uint8_buf_t calibration_mode;        // offset 55, 1 byte
  big_uint8_buf_t window_blockage_status;  // offset 56, 1 byte
  uint8_t window_blockage_level[18];       // offset 57, 18 bytes
  uint8_t reserved2[14];                   // offset 75, 14 bytes
  big_uint8_buf_t time_sync_mode;          // offset 89, 1 byte
  big_uint8_buf_t time_sync_status;        // offset 90, 1 byte
  Timestamp time;                          // offset 91, 10 bytes
  big_uint8_buf_t phy_mode;                // offset 101, 1 byte
  IpAddress src_ip;                        // offset 102, 4 bytes
  IpAddress net_mask;                      // offset 106, 4 bytes
  MacAddress mac_address;                  // offset 110, 6 bytes
  IpAddress msop_dst_ip;                   // offset 116, 4 bytes
  big_uint16_buf_t msop_src_port;          // offset 120, 2 bytes
  big_uint16_buf_t msop_dst_port;          // offset 122, 2 bytes
  IpAddress difop1_dst_ip;                 // offset 124, 4 bytes
  big_uint16_buf_t difop1_src_port;        // offset 128, 2 bytes
  big_uint16_buf_t difop1_dst_port;        // offset 130, 2 bytes
  uint8_t reserved3[24];                   // offset 132, 24 bytes
  uint8_t tmon6_win;                       // offset 156, 1 byte
  uint8_t tmon8_fpga;                      // offset 157, 1 byte
  uint8_t reserved4[71];                   // offset 158, 71 bytes
  big_uint8_buf_t lidar_function_fault;    // offset 229, 1 byte
  big_uint8_buf_t ext_power_supply_fault;  // offset 230, 1 byte
  big_uint16_buf_t comm_fault;             // offset 231, 2 bytes
  big_uint8_buf_t fault_level;             // offset 233, 1 byte
  big_uint16_buf_t fault_id;               // offset 234, 2 bytes
  big_uint32_buf_t fault_value;            // offset 236, 4 bytes
  uint8_t dtc_list[4];                     // offset 240, 4 bytes
  uint8_t e2e[12];                         // offset 244, 12 bytes
};  // total: 256 bytes

#pragma pack(pop)
}  // namespace robosense_packet::emx

namespace robosense_packet
{
/// @brief Specialization for EMX as it doesn't have range_resolution in header
template <>
inline double get_dis_unit<robosense_packet::emx::Packet>(const robosense_packet::emx::Packet &)
{
  return 0.005;
}
}  // namespace robosense_packet

class EMX : public RobosenseSensorDirectional<
              robosense_packet::emx::Packet, robosense_packet::emx::InfoPacket>
{
private:
  static constexpr uint8_t sync_mode_internal_flag = 0x00;
  static constexpr uint8_t sync_mode_pps_flag = 0x01;
  static constexpr uint8_t sync_mode_e2e_flag = 0x02;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;
  static constexpr uint8_t sync_mode_p2p_flag = 0x04;

  // EMX DIFOP wave mode byte values
  static constexpr uint8_t wave_mode_nearest_farthest = 0x00;
  static constexpr uint8_t wave_mode_strongest = 0x04;
  static constexpr uint8_t wave_mode_farthest = 0x05;
  static constexpr uint8_t wave_mode_nearest = 0x06;

  static constexpr int VECTOR_BASE = 32768;

public:
  static constexpr bool has_custom_projection = true;
  typedef AngleCorrector angle_corrector_t;
  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 288000;

  ReturnMode get_return_mode(const robosense_packet::emx::InfoPacket & info_packet) override
  {
    switch (info_packet.wave_mode.value()) {
      case wave_mode_nearest_farthest:
        return ReturnMode::DUAL;
      case wave_mode_strongest:
        return ReturnMode::SINGLE_STRONGEST;
      case wave_mode_farthest:
        return ReturnMode::SINGLE_LAST;
      case wave_mode_nearest:
        return ReturnMode::SINGLE_FIRST;
      default:
        return ReturnMode::UNKNOWN;
    }
  }

  RobosenseCalibrationConfiguration get_sensor_calibration(
    const robosense_packet::emx::InfoPacket & /*info_packet*/) override
  {
    return {};
  }

  bool get_sync_status(const robosense_packet::emx::InfoPacket & info_packet) override
  {
    return info_packet.time_sync_status.value() == 0x01;
  }

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::emx::InfoPacket & info_packet) override
  {
    std::map<std::string, std::string> sensor_info;

    switch (info_packet.time_sync_mode.value()) {
      case sync_mode_internal_flag:
        sensor_info["time_sync_mode"] = "internal";
        break;
      case sync_mode_pps_flag:
        sensor_info["time_sync_mode"] = "pps";
        break;
      case sync_mode_e2e_flag:
        sensor_info["time_sync_mode"] = "e2e";
        break;
      case sync_mode_gptp_flag:
        sensor_info["time_sync_mode"] = "gptp";
        break;
      case sync_mode_p2p_flag:
        sensor_info["time_sync_mode"] = "p2p";
        break;
      default:
        sensor_info["time_sync_mode"] = "n/a";
        break;
    }

    populate_sync_status_info(sensor_info, info_packet.time_sync_status.value());
    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());

    // Network config from DIFOP
    sensor_info["sensor_ip"] = info_packet.src_ip.to_string();
    sensor_info["dest_ip"] = info_packet.msop_dst_ip.to_string();
    sensor_info["msop_dst_port"] = std::to_string(info_packet.msop_dst_port.value());
    sensor_info["difop_dst_port"] = std::to_string(info_packet.difop1_dst_port.value());
    return sensor_info;
  }

  int get_packet_relative_point_time_offset(
    const robosense_packet::emx::Packet & packet, const uint32_t block_id,
    const uint32_t /*channel_id*/,
    const std::shared_ptr<const RobosenseSensorConfiguration> & /*sensor_configuration*/) override
  {
    return static_cast<int>(packet.body.blocks[block_id].time_offset) * 1000;
  }

  template <typename CorrectorT>
  void populate_point_xyz(
    ::nebula::drivers::NebulaPoint & point, const robosense_packet::emx::Packet & pkt,
    uint32_t block_id, uint32_t channel_id, CorrectorT & /*angle_corrector*/)
  {
    const auto & unit = pkt.body.blocks[block_id].units[channel_id];

    // RSMX provides x, y, z direction vectors
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
