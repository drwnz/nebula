// Copyright 2024 TIER IV, Inc.
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

#include <nebula_robosense_decoders/robosense_plugin.hpp>

#include <iostream>

namespace nebula::drivers
{

RobosenseSensorDecoderRuntime::RobosenseSensorDecoderRuntime()
{
}

void RobosenseSensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  config_ = config;
  
  auto r_config = std::make_shared<RobosenseSensorConfiguration>();
  r_config->sensor_model = config.sensor_model;
  r_config->host_ip = config.host_ip;
  r_config->sensor_ip = config.sensor_ip;
  r_config->data_port = config.data_port;
  r_config->gnss_port = config.gnss_port;
  r_config->frame_id = config.frame_id;
  
  // Base class fields
  r_config->min_range = config.min_range;
  r_config->max_range = config.max_range;
  r_config->return_mode = config.return_mode;

  auto c_config = std::make_shared<RobosenseCalibrationConfiguration>();
  if (!config.calibration_file.empty()) {
      c_config->load_from_file(config.calibration_file);
  }
  
  driver_ = std::make_unique<RobosenseDriver>(r_config, c_config);
  info_driver_ = std::make_unique<RobosenseInfoDriver>(r_config);
}

void RobosenseSensorDecoderRuntime::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
}

void RobosenseSensorDecoderRuntime::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void RobosenseSensorDecoderRuntime::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
}

SensorPacketResult RobosenseSensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  progress_.processed_packets++;

  if (packet.channel == SensorPacketChannel::Data) {
    if (!driver_) return SensorPacketResult::Error;
    progress_.matched_packets++;
    auto [pointcloud, timestamp_s] = driver_->parse_cloud_packet(packet.payload);
    progress_.decoded_packets++;
    if (pointcloud) {
      progress_.output_count++;
      if (output_callback_) {
        SensorDecodedOutput output;
        output.kind = SensorOutputKind::PointCloud;
        output.timestamp_ns = static_cast<uint64_t>(timestamp_s * 1e9);
        output.sensor_id = "robosense";
        output.payload = pointcloud;
        output_callback_(output);
      }
    }
    if (progress_callback_) progress_callback_(progress_);
    return SensorPacketResult::Success;
  } else if (packet.channel == SensorPacketChannel::Info) {
    if (!info_driver_) return SensorPacketResult::Error;
    progress_.matched_packets++;
    if (info_driver_->decode_info_packet(packet.payload) == Status::OK) {
        progress_.decoded_packets++;
        if (output_callback_) {
            SensorDecodedOutput output;
            output.kind = SensorOutputKind::Status;
            output.timestamp_ns = packet.timestamp_ns;
            output.sensor_id = "robosense";
            output.payload = info_driver_->get_sensor_info();
            output_callback_(output);
        }
        if (progress_callback_) progress_callback_(progress_);
        return SensorPacketResult::Success;
    }
  }

  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

void RobosenseSensorDecoderRuntime::flush()
{
}

void RobosenseSensorDecoderRuntime::on_pointcloud([[maybe_unused]] const NebulaPointCloudPtr & pointcloud, [[maybe_unused]] double timestamp_s)
{
}

SensorPluginMetadata RobosenseSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "robosense";
  md.package_name = "nebula_robosense_decoders";
  md.library_path = "libnebula_robosense_decoders_plugin.so";
  md.factory_symbol = "create_nebula_sensor_plugin";
  md.supported_models = {
    SensorModel::ROBOSENSE_HELIOS,
    SensorModel::ROBOSENSE_BPEARL_V3,
    SensorModel::ROBOSENSE_BPEARL_V4,
    SensorModel::ROBOSENSE_E1,
    SensorModel::ROBOSENSE_EM4,
    SensorModel::ROBOSENSE_EMX
  };
  return md;
}

std::vector<SensorModelInfo> RobosenseSensorPlugin::supported_models() const
{
  return {
    {SensorModel::ROBOSENSE_HELIOS, "Helios", "Robosense Helios"},
    {SensorModel::ROBOSENSE_BPEARL_V3, "Bpearl_V3", "Robosense Bpearl V3.0"},
    {SensorModel::ROBOSENSE_BPEARL_V4, "Bpearl_V4", "Robosense Bpearl V4.0"},
    {SensorModel::ROBOSENSE_E1, "E1", "Robosense E1"},
    {SensorModel::ROBOSENSE_EM4, "EM4", "Robosense EM4"},
    {SensorModel::ROBOSENSE_EMX, "EMX", "Robosense EMX"}
  };
}

std::vector<PacketChannelRequirement> RobosenseSensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  std::vector<PacketChannelRequirement> reqs;
  
  // MSOP
  PacketChannelRequirement msop;
  msop.transport = SensorTransportKind::UDP;
  msop.channel = SensorPacketChannel::Data;
  msop.required = true;
  msop.udp_destination_port = config.data_port;
  msop.payload_signature = "55aa"; // Standard Robosense MSOP header
  reqs.push_back(msop);

  // DIFOP
  PacketChannelRequirement difop;
  difop.transport = SensorTransportKind::UDP;
  difop.channel = SensorPacketChannel::Info;
  difop.required = true;
  difop.udp_destination_port = config.gnss_port != 0 ? config.gnss_port : 7788;
  difop.payload_signature = "a55a"; // Standard Robosense DIFOP header
  reqs.push_back(difop);

  return reqs;
}

std::vector<LiveTransportRequirement> RobosenseSensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  std::vector<LiveTransportRequirement> reqs;
  
  LiveTransportRequirement msop;
  msop.transport = SensorTransportKind::UDP;
  msop.channel = SensorPacketChannel::Data;
  msop.required = true;
  msop.name = "robosense_msop";
  msop.port = config.data_port;
  reqs.push_back(msop);

  LiveTransportRequirement difop;
  difop.transport = SensorTransportKind::UDP;
  difop.channel = SensorPacketChannel::Info;
  difop.required = true;
  difop.name = "robosense_difop";
  difop.port = config.gnss_port != 0 ? config.gnss_port : 7788;
  reqs.push_back(difop);

  return reqs;
}

std::unique_ptr<SensorDecoderRuntime> RobosenseSensorPlugin::create_decoder_runtime() const
{
  return std::make_unique<RobosenseSensorDecoderRuntime>();
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::RobosenseSensorPlugin();
}
}
