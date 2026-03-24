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
#include "nebula_robosense_common/robosense_common.hpp"
#include "nebula_robosense_decoders/decoders/angle_corrector.hpp"
#include "nebula_robosense_decoders/decoders/angle_corrector_emx.hpp"
#include "nebula_robosense_decoders/decoders/robosense_packet.hpp"
#include "nebula_robosense_decoders/decoders/robosense_sensor.hpp"

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
  uint8_t header_id[4];               // 0
  big_uint16_buf_t pkt_seq;           // 4
  big_uint16_buf_t protocol_version;  // 6
  big_uint8_buf_t return_mode;        // 8
  big_uint8_buf_t time_mode;          // 9
  Timestamp timestamp;                // 10
  big_uint8_buf_t fram_sync;          // 20
  big_uint8_buf_t frame_rate;         // 21
  big_uint16_buf_t column_num;        // 22
  big_int16_buf_t yaw_angle;          // 24
  big_uint8_buf_t pack_mode;          // 26
  big_uint8_buf_t surface_id;         // 27
  big_uint16_buf_t temperature;       // 28
  big_uint8_buf_t lidar_type;         // 30
  big_uint8_buf_t reserved;           // 31
};

struct Unit
{
  big_uint16_buf_t distance;        // 0
  big_uint8_buf_t reflectivity;     // 2
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
  Block blocks[192];
};

struct Footer
{
  big_uint16_buf_t data_length;
  big_uint16_buf_t counter;
  big_uint32_buf_t data_id;
  big_uint32_buf_t crc32;
};

struct Packet : public robosense_packet::PacketBase<192, 1, 2, 512>
{
  typedef Body body_t;
  Header header;
  body_t body;
  Footer footer;
};

struct InfoEthernet
{
  uint8_t data[62];
};

struct InfoFaults
{
  uint8_t data[16];
};

struct InfoPacket
{
  uint8_t id[8];
  uint8_t sw_ver[4];
  uint8_t AIPn[4];
  uint8_t PIPn[4];
  uint8_t mcu_ver[4];
  uint8_t hw_ver;
  uint8_t int_sn[6];
  uint8_t _sn[4];
  uint8_t reserved_0[9];
  InfoEthernet ether;
  big_uint8_buf_t work_mode;
  big_uint8_buf_t frame_rate;
  big_uint8_buf_t wave_mode;
  uint8_t time_info[8];
  big_uint8_buf_t master_slave_mode;
  big_uint8_buf_t surface_id;
  int8_t yaw_offset[26];
  big_int16_buf_t pitch_angle[192];
  big_int16_buf_t roll_offset;
  uint8_t voltage_temp[66];
  uint8_t reserved2[5];
  InfoFaults faults;
  uint8_t reserved3[28];
  big_uint16_buf_t data_length;
  big_uint16_buf_t counter;
  big_uint32_buf_t data_id;
  big_uint32_buf_t crc32;
};

struct InfoPacket2
{
  uint8_t id[4];
  uint8_t reserved0[63];
  big_uint8_buf_t surface_id;
  big_uint8_buf_t pixelCnt;
  big_uint8_buf_t vcselCnt;
  int8_t yaw_offset[24];
  big_int16_buf_t pitch_angle[192];
  big_int16_buf_t surface_pitch_offset[2];
  big_int16_buf_t roll_offset;
  uint8_t reserved1[4];
  big_uint16_buf_t data_length;
  big_uint16_buf_t counter;
  big_uint32_buf_t data_id;
  big_uint32_buf_t crc32;
};

struct InfoPacket256
{
  uint8_t id[8];             // offset 0, 8 bytes
  uint8_t res0[8];           // offset 8, 8 bytes
  uint8_t sw_version[3];     // offset 16, 3 bytes
  uint8_t res1[1];           // offset 19, 1 byte
  uint8_t sn[6];             // offset 20, 6 bytes
  uint8_t res2[18];          // offset 26, 18 bytes
  IpAddress local_ip;        // offset 44, 4 bytes
  IpAddress net_mask;        // offset 48, 4 bytes
  MacAddress mac_address;    // offset 52, 6 bytes
  IpAddress msop_remote_ip;  // offset 58, 4 bytes
  big_uint16_buf_t msop_local_port;
  big_uint16_buf_t msop_remote_port;
  IpAddress difop_remote_ip;
  big_uint16_buf_t difop_local_port;
  big_uint16_buf_t difop_remote_port;
  uint8_t res3[25];
  big_uint8_buf_t frequency_setting;
  big_uint8_buf_t return_mode;
  big_uint8_buf_t time_mode;
  big_uint8_buf_t time_sync_status;
  Timestamp time;
  big_uint8_buf_t phy_mode;
  uint8_t res4[142];
};

struct CombinedInfo
{
  InfoPacket packet1;
  InfoPacket2 packet2;
  InfoPacket256 packet256;
  bool packet1_received{false};
  bool packet2_received{false};
  bool packet256_received{false};
};

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
              robosense_packet::emx::Packet, robosense_packet::emx::CombinedInfo>
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
  typedef AngleCorrectorEMX angle_corrector_t;
  static constexpr float min_range = 0.5f;
  static constexpr float max_range = 350.f;
  static constexpr size_t max_scan_buffer_points = 288000;

  ReturnMode get_return_mode(const robosense_packet::emx::CombinedInfo & info_packet) override
  {
    if (info_packet.packet1_received) {
      switch (info_packet.packet1.wave_mode.value()) {
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
    } else if (info_packet.packet256_received) {
      uint8_t rm = info_packet.packet256.return_mode.value();
      if (rm == 0x00) return ReturnMode::SINGLE_LAST;
      if (rm == 0x04) return ReturnMode::SINGLE_STRONGEST;
      if (rm == 0x07) return ReturnMode::SINGLE_FIRST;
      if (rm >= 0x09) return ReturnMode::DUAL;
      return ReturnMode::SINGLE_STRONGEST;  // Default
    }
    return ReturnMode::UNKNOWN;
  }

  RobosenseCalibrationConfiguration get_sensor_calibration(
    const robosense_packet::emx::CombinedInfo & info) override
  {
    RobosenseCalibrationConfiguration calib;
    calib.set_channel_size(192);
    calib.calibration.clear();
    calib.calibration.resize(192);
    calib.pixel_pitch.assign(192, 0);
    calib.half_vcsel_yaw_offset.assign(24, 0);
    calib.surface_pitch_offset.assign(2, 0);

    if (info.packet1_received) {
      for (size_t i = 0; i < 192; ++i) {
        calib.pixel_pitch[i] = info.packet1.pitch_angle[i].value();
      }
    }
    if (info.packet2_received) {
      for (size_t i = 0; i < 24; ++i) {
        calib.half_vcsel_yaw_offset[i] = info.packet2.yaw_offset[i];
      }
      for (size_t i = 0; i < 2; ++i) {
        calib.surface_pitch_offset[i] = info.packet2.surface_pitch_offset[i].value();
      }
    }
    return calib;
  }

  bool get_sync_status(const robosense_packet::emx::CombinedInfo & info_packet) override
  {
    if (info_packet.packet1_received) {
      return info_packet.packet1.work_mode.value() == 0x01;
    } else if (info_packet.packet256_received) {
      return info_packet.packet256.time_sync_status.value() == 0x01;
    }
    return false;
  }

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::emx::CombinedInfo & info_packet) override
  {
    std::map<std::string, std::string> sensor_info;
    if (info_packet.packet256_received) {
      sensor_info["sn"] = std::string((char *)info_packet.packet256.sn, 6);
      sensor_info["sensor_ip"] = info_packet.packet256.local_ip.to_string();
    }
    return sensor_info;
  }

  int get_packet_relative_point_time_offset(
    const robosense_packet::emx::Packet & /*packet*/, const uint32_t /*block_id*/,
    const uint32_t /*channel_id*/,
    const std::shared_ptr<const RobosenseSensorConfiguration> & /*sensor_configuration*/) override
  {
    return 0;  // EMX doesn't have block-level time offsets in the new MSOP struct
  }

  template <typename CorrectorT>
  void populate_point_xyz(
    ::nebula::drivers::NebulaPoint & point, const robosense_packet::emx::Packet & pkt,
    uint32_t block_id, uint32_t /*channel_id*/, CorrectorT & angle_corrector)
  {
    uint8_t surface_id = pkt.header.surface_id.value();
    uint8_t mirror_id = (surface_id >= 2) ? (surface_id - 1) : surface_id;
    if (mirror_id > 1) mirror_id = 0;

    int16_t raw_yaw = pkt.header.yaw_angle.value();

    // Decode dual return offset logic
    uint16_t pkt_seq = pkt.header.pkt_seq.value() - 1;
    bool is_dual_mode = (pkt.header.return_mode.value() == 0);
    uint16_t req_offset = (is_dual_mode && (pkt_seq % 2 != 0)) ? 192 : 0;

    uint16_t real_chan;
    if (is_dual_mode) {
      uint16_t real_blk = block_id + req_offset;
      real_chan = real_blk / 2;  // dual_return_pitch_index mapping 0, 0, 1, 1, ...
    } else {
      real_chan = block_id;
    }

    auto corrected_data =
      angle_corrector.get_corrected_angle_data_emx(raw_yaw, real_chan, mirror_id);

    float xy_dist = point.distance * corrected_data.cos_elevation;
    point.x = xy_dist * corrected_data.cos_azimuth;
    point.y = xy_dist * corrected_data.sin_azimuth;
    point.z = point.distance * corrected_data.sin_elevation;

    point.azimuth = corrected_data.azimuth_rad;
    point.elevation = corrected_data.elevation_rad;
    point.channel = static_cast<uint16_t>(real_chan);
  }
};

}  // namespace nebula::drivers
