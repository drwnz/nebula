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
namespace robosense_packet::emx
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
  uint8_t reserved[10];
  big_uint8_buf_t lidar_type;
  big_uint8_buf_t temperature;
};

struct Unit
{
  big_uint16_buf_t distance;
  big_uint16_buf_t radius_sd;
  big_int16_buf_t x;
  big_int16_buf_t y;
  big_int16_buf_t z;
  big_uint8_buf_t reflectivity;
  big_uint8_buf_t intensity_sd;
  big_uint8_buf_t point_attribute;
};

struct Block
{
  typedef Unit unit_t;
  big_uint8_buf_t time_offset;
  Unit units[2];
};

struct Body
{
  typedef Block block_t;
  Block blocks[50];
};

struct Packet : public PacketBase<2, 50, 2, 2>
{
  typedef Body body_t;
  Header header;
  body_t body;
  uint8_t reserved[16];
  uint8_t crc32[4];
  uint8_t rolling_counter[2];
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

class EMX : public RobosenseSensorDirectional<
              robosense_packet::emx::Packet, robosense_packet::emx::InfoPacket>
{
private:
  static constexpr uint8_t sync_mode_internal_flag = 0x00;
  static constexpr uint8_t sync_mode_pps_flag = 0x01;
  static constexpr uint8_t sync_mode_e2e_flag = 0x02;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;
  static constexpr uint8_t sync_mode_p2p_flag = 0x04;

  // EMX DIFOP wave mode byte values (from user manual)
  static constexpr uint8_t wave_mode_nearest_farthest = 0x00;
  static constexpr uint8_t wave_mode_strongest = 0x04;
  static constexpr uint8_t wave_mode_farthest = 0x05;
  static constexpr uint8_t wave_mode_nearest = 0x06;

public:
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
};
}  // namespace nebula::drivers
