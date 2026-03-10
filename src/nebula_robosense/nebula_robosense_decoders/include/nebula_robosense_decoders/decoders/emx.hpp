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
  big_uint64_buf_t header;
  uint8_t reserved1[30];
  uint8_t sn[16];
  uint8_t reserved2[202];
};

struct EmxInfoPacket
{
  big_uint32_buf_t status_hdr;
  uint8_t reserved[20];
  uint8_t sw_version[3];
  uint8_t hw_version[2];
  uint8_t reserved1[6];
  uint8_t customer_sn[16];
  big_uint8_buf_t work_mode;
  big_uint8_buf_t frame_rate;
  big_uint8_buf_t wave_mode;
  big_uint8_buf_t roi_mode;
  big_uint8_buf_t calibration_mode;
  big_uint8_buf_t window_blockage_status;
  uint8_t window_blockage_level[18];
  uint8_t reserved2[14];
  big_uint8_buf_t time_sync_mode;
  big_uint8_buf_t time_sync_status;
  Timestamp time;
  big_uint8_buf_t phy_mode;
  uint8_t src_ip[4];
  uint8_t net_mask[4];
  uint8_t mac_address[6];
  uint8_t msop_dst_ip[4];
  big_uint16_buf_t msop_src_port;
  big_uint16_buf_t msop_dst_port;
  uint8_t difop1_dst_ip[4];
  big_uint16_buf_t difop1_src_port;
  big_uint16_buf_t difop1_dst_port;
  uint8_t reserved3[24];
  uint8_t tmon6_win;
  uint8_t tmon8_fpga;
  uint8_t reserved4[71];
  big_uint8_buf_t lidar_function_fault;
  big_uint8_buf_t ext_power_supply_fault;
  big_uint16_buf_t comm_fault;
  big_uint8_buf_t fault_level;
  big_uint16_buf_t fault_id;
  big_uint32_buf_t fault_value;
  uint8_t dtc_list[4];
  uint8_t e2e[12];
};

#pragma pack(pop)
}  // namespace robosense_packet::emx

class EMX : public RobosenseSensorDirectional<
              robosense_packet::emx::Packet, robosense_packet::emx::EmxInfoPacket>
{
private:
  static constexpr uint8_t sync_mode_internal_flag = 0x00;
  static constexpr uint8_t sync_mode_pps_flag = 0x01;
  static constexpr uint8_t sync_mode_e2e_flag = 0x02;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;
  static constexpr uint8_t sync_mode_p2p_flag = 0x04;

public:
  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 288000;

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::emx::EmxInfoPacket & info_packet) override
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
    return sensor_info;
  }
};
}  // namespace nebula::drivers
