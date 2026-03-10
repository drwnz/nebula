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
namespace robosense_packet::em4
{
#pragma pack(push, 1)

struct Header
{
  big_uint32_buf_t header_id;
  big_uint16_buf_t pkt_seq;
  uint8_t reserved_1;
  uint8_t reserved_2;
  big_uint8_buf_t return_mode;
  big_uint8_buf_t time_mode;
  Timestamp timestamp;
  big_uint8_buf_t frame_sync;
  big_uint8_buf_t frame_rate;
  big_uint16_buf_t column_num;
  big_int16_buf_t yaw_angle;
  uint8_t reserved_3;
  big_uint8_buf_t surface_id;
  big_int8_buf_t vcsel_interval;
  uint8_t reserved_4;
  big_uint8_buf_t lidar_type;
  big_uint8_buf_t main_temp;
};

struct Unit
{
  big_uint16_buf_t distance;
  big_uint8_buf_t reflectivity;
  big_uint8_buf_t point_attribute;
};

struct Block
{
  typedef Unit unit_t;
  Unit units[260];
};

struct Body
{
  typedef Block block_t;
  Block blocks[1];
};

struct Packet : public PacketBase<1, 260, 1, 1>
{
  typedef Body body_t;
  Header header;
  body_t body;
  big_uint16_buf_t data_length;
  big_uint16_buf_t counter;
  big_uint32_buf_t data_id;
  big_uint32_buf_t crc32;
};

struct InfoPacket
{
  big_uint32_buf_t status_hdr;
  uint8_t reserved1[20];
  uint8_t sw_version[3];
  uint8_t hw_version[2];
  uint8_t int_sn[6];
  uint8_t cus_sn[16];
  uint8_t reserved2;
  big_uint8_buf_t frame_rate;
  big_uint8_buf_t wave_mode;
  uint8_t reserved3[10];
  big_uint8_buf_t lidar_heater_status;
  uint8_t reserved4[24];
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
  uint8_t difop2_dst_ip[4];
  big_uint16_buf_t difop2_src_port;
  big_uint16_buf_t difop2_dst_port;
  uint8_t doip_dst_ip[4];
  big_uint16_buf_t doip_src_port;
  uint8_t reserved5[10];
  big_uint16_buf_t mcu_vmon_rx_d1v1;
  big_uint16_buf_t mcu_vmon_f_1v0;
  big_uint16_buf_t mcu_vmon_f_1v8;
  big_uint16_buf_t mcu_vmon_f_2v5;
  big_uint16_buf_t mcu_vmon_m_3v3;
  big_uint16_buf_t mcu_vmon_a_3v3;
  big_uint16_buf_t mcu_vmon_wake_ext;
  big_uint16_buf_t mcu_imon_window;
  big_uint16_buf_t mcu_vmon_window;
  big_uint16_buf_t mcu_vmom_sys_5v;
  big_uint16_buf_t mcu_vmom_vin;
  big_uint16_buf_t pl_vmom_m_1v2;
  big_uint16_buf_t pl_vmon_chg;
  big_uint16_buf_t pl_vmon_vop;
  big_uint16_buf_t rx_vt4_n;
  big_uint16_buf_t rx_3v3;
  uint8_t res3[4];
  big_uint8_buf_t temp_rx_sensor;
  big_uint8_buf_t temp_fpga1;
  big_uint8_buf_t temp_mcu;
  big_uint8_buf_t temp_motor;
  big_uint8_buf_t temp_fpga2;
  big_uint8_buf_t temp_txr1;
  big_uint8_buf_t temp_rx;
  big_uint8_buf_t temp_window;
  big_uint8_buf_t temp_txr2;
  uint8_t reserved6[5];
  big_uint8_buf_t humidity_sensor_value;
  big_uint8_buf_t temperature_sensor_value;
  big_uint8_buf_t dew_point;
  uint8_t reserved7[7];
  uint8_t internal_power_supply_fault[3];
  uint8_t lidar_temp_fault[3];
  uint8_t internal_software_fault[3];
  uint8_t internal_performance_fault[4];
  big_uint8_buf_t lidar_function_fault;
  big_uint8_buf_t ext_power_supply_fault;
  big_uint16_buf_t ext_comm_fault;
  uint8_t fault_status_reserved[11];
  big_uint16_buf_t e2e_data_length;
  big_uint16_buf_t e2e_counter;
  big_uint32_buf_t e2e_data_id;
  big_uint32_buf_t e2e_crc32;
};

struct InfoPacket2
{
  big_uint32_buf_t info_hdr;
  uint8_t reserved_0[63];
  big_uint8_buf_t surface_cnt;
  big_uint8_buf_t half_vcsel_pixel_cnt;
  big_uint8_buf_t half_vcsel_cnt;
  big_int16_buf_t half_vcsel_yaw_offset[13];
  big_int16_buf_t pixel_pitch[520];
  big_int16_buf_t surface_pitch_offset[4];
  uint8_t reserved_1[6];
  big_uint16_buf_t e2e_data_length;
  big_uint16_buf_t e2e_counter;
  big_uint32_buf_t e2e_data_id;
  big_uint32_buf_t e2e_crc32;
};

#pragma pack(pop)
}  // namespace robosense_packet::em4

class EM4 : public RobosenseSensorDirectional<
              robosense_packet::em4::Packet, robosense_packet::em4::InfoPacket>
{
private:
  static constexpr uint8_t sync_mode_internal_flag = 0x00;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;

public:
  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 300.f;
  static constexpr size_t max_scan_buffer_points = 1248000;

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::em4::InfoPacket & info_packet) override
  {
    std::map<std::string, std::string> sensor_info;

    switch (info_packet.time_sync_mode.value()) {
      case sync_mode_internal_flag:
        sensor_info["time_sync_mode"] = "internal";
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
