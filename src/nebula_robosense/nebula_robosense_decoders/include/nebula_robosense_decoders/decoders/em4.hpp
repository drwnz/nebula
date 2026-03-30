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
#include "nebula_robosense_decoders/decoders/angle_corrector_em4.hpp"
#include "nebula_robosense_decoders/decoders/robosense_packet.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor_directional.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

using namespace boost::endian;  // NOLINT(build/namespaces)

namespace nebula::drivers
{
namespace robosense_packet::em4
{
#pragma pack(push, 1)

struct Header
{
  big_uint32_buf_t header_id;   // 0
  big_uint16_buf_t pkt_seq;     // 4
  uint8_t reserved1[2];         // 6
  big_uint8_buf_t return_mode;  // 8
  big_uint8_buf_t time_mode;    // 9
  Timestamp timestamp;          // 10
  big_uint8_buf_t frame_sync;   // 20
  big_uint8_buf_t frame_rate;   // 21
  big_uint16_buf_t column_num;  // 22
  big_int16_buf_t yaw_angle;    // 24
  big_uint8_buf_t pack_mode;    // 26
  big_uint8_buf_t surface_id;   // 27
  big_uint16_buf_t reserved2;   // 28
  big_uint8_buf_t lidar_type;   // 30
  big_uint8_buf_t temperature;  // 31
};

struct Unit
{
  big_uint16_buf_t distance;        // 0 (Radius)
  big_uint8_buf_t reflectivity;     // 2 (Intensity)
  big_uint8_buf_t point_attribute;  // 3
};

struct Block
{
  typedef Unit unit_t;
  Unit units[1];
};

struct Body
{
  typedef Block block_t;
  Block blocks[260];
};

struct Footer
{
  big_uint16_buf_t data_length;  // 1072
  big_uint16_buf_t counter;      // 1074
  big_uint32_buf_t data_id;      // 1076
  big_uint32_buf_t crc32;        // 1080
};

struct Packet : public robosense_packet::PacketBase<260, 1, 2, 520>
{
  typedef Body body_t;
  Header header;
  body_t body;
  Footer footer;
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
  IpAddress src_ip;
  IpAddress net_mask;
  MacAddress mac_address;
  IpAddress msop_dst_ip;
  big_uint16_buf_t msop_src_port;
  big_uint16_buf_t msop_dst_port;
  IpAddress difop1_dst_ip;
  big_uint16_buf_t difop1_src_port;
  big_uint16_buf_t difop1_dst_port;
  IpAddress difop2_dst_ip;
  big_uint16_buf_t difop2_src_port;
  big_uint16_buf_t difop2_dst_port;
  IpAddress doip_dst_ip;
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

struct CombinedInfo
{
  InfoPacket packet1;
  InfoPacket2 packet2;
  bool packet1_received{false};
  bool packet2_received{false};
};

#pragma pack(pop)
}  // namespace robosense_packet::em4

class EM4 : public RobosenseSensorDirectional<
              robosense_packet::em4::Packet, robosense_packet::em4::CombinedInfo>
{
private:
  static constexpr uint8_t sync_mode_internal_flag = 0x00;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;

public:
  static constexpr bool has_custom_projection = true;
  typedef AngleCorrectorEM4 angle_corrector_t;
  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 300.f;
  static constexpr size_t max_scan_buffer_points = 1248000;

  ReturnMode get_return_mode(const robosense_packet::em4::CombinedInfo & info) override
  {
    if (!info.packet1_received) return ReturnMode::UNKNOWN;
    return return_mode_from_wave_mode(info.packet1.wave_mode.value());
  }

  RobosenseCalibrationConfiguration get_sensor_calibration(
    const robosense_packet::em4::CombinedInfo & info) override
  {
    RobosenseCalibrationConfiguration calib;
    calib.set_channel_size(520);
    if (info.packet2_received) {
      for (size_t i = 0; i < 13; ++i) {
        calib.half_vcsel_yaw_offset[i] = info.packet2.half_vcsel_yaw_offset[i].value();
      }
      for (size_t i = 0; i < 520; ++i) {
        calib.pixel_pitch[i] = info.packet2.pixel_pitch[i].value();
      }
      for (size_t i = 0; i < 4; ++i) {
        calib.surface_pitch_offset[i] = info.packet2.surface_pitch_offset[i].value();
      }
    }
    return calib;
  }

  bool get_sync_status(const robosense_packet::em4::CombinedInfo & info) override
  {
    return info.packet1_received && info.packet1.time_sync_status.value() == 0x01;
  }

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::em4::CombinedInfo & info) override
  {
    std::map<std::string, std::string> sensor_info;
    if (!info.packet1_received) return sensor_info;

    const auto & packet1 = info.packet1;
    switch (packet1.time_sync_mode.value()) {
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

    populate_sync_status_info(sensor_info, packet1.time_sync_status.value());
    sensor_info["time"] = std::to_string(packet1.time.get_time_in_ns());

    sensor_info["sensor_ip"] = packet1.src_ip.to_string();
    sensor_info["dest_ip"] = packet1.msop_dst_ip.to_string();
    sensor_info["msop_dst_port"] = std::to_string(packet1.msop_dst_port.value());
    sensor_info["difop_dst_port"] = std::to_string(packet1.difop1_dst_port.value());
    return sensor_info;
  }

  int get_packet_relative_point_time_offset(
    const robosense_packet::em4::Packet & packet, const uint32_t /*block_id*/,
    const uint32_t channel_id,
    const std::shared_ptr<const RobosenseSensorConfiguration> & /*sensor_configuration*/) override
  {
    (void)packet;
    (void)channel_id;
    return 0;
  }

  template <typename CorrectorT>
  void populate_point_xyz(
    ::nebula::drivers::NebulaPoint & point, const robosense_packet::em4::Packet & pkt,
    uint32_t block_id, uint32_t /*channel_id*/, CorrectorT & angle_corrector)
  {
    // The validated vendor output for the currently supported EM4 packet format is equivalent to
    // using surface index 0.
    // Do not use byte 27 surface_id here: it regresses the projection against the vendor PCDs.
    // Do not infer that byte 20 is the true surface selector either; the current evidence only
    // supports effective surface-0 behavior for this EM4 format/firmware.
    constexpr uint8_t surface_id = 0;

    int16_t raw_yaw = pkt.header.yaw_angle.value();

    uint16_t pkt_seq = pkt.header.pkt_seq.value() - 1;
    bool is_dual_mode = (pkt.header.return_mode.value() == 0);
    const uint16_t seq_mod = is_dual_mode ? (pkt_seq % 4) : (pkt_seq % 2);

    uint16_t real_chan;
    if (is_dual_mode) {
      real_chan = static_cast<uint16_t>(block_id / 2) + seq_mod * 130;
    } else {
      real_chan = static_cast<uint16_t>(block_id + seq_mod * 260);
    }

    auto corrected_data =
      angle_corrector.get_corrected_angle_data_em4(raw_yaw, real_chan, surface_id);
    populate_point_from_corrected_angle(point, corrected_data, static_cast<uint16_t>(real_chan));
  }
};

}  // namespace nebula::drivers
