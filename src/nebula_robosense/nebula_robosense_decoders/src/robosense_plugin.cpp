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
  r_config->frame_id = config.frame_id;
  
  // Defaults
  r_config->gnss_port = 7788;
  r_config->scan_phase = 0;

  auto c_config = std::make_shared<RobosenseCalibrationConfiguration>();
  // Placeholder for calibration loading
  
  driver_ = std::make_unique<RobosenseDriver>(r_config, c_config);
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
  if (!driver_) {
    return SensorPacketResult::Error;
  }

  progress_.processed_packets++;

  if (packet.channel == SensorPacketChannel::Data || packet.channel == SensorPacketChannel::Unknown) {
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
    // This is handled in process_packet directly for Robosense since parse_cloud_packet returns it
}

SensorPluginMetadata RobosenseSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "robosense";
  md.package_name = "nebula_robosense_decoders";
  md.library_path = "libnebula_robosense_decoders_plugin.so";
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
    {SensorModel::ROBOSENSE_BPEARL_V3, "Bpearl V3.0", "Robosense Bpearl V3.0"},
    {SensorModel::ROBOSENSE_BPEARL_V4, "Bpearl V4.0", "Robosense Bpearl V4.0"},
    {SensorModel::ROBOSENSE_E1, "E1", "Robosense E1"},
    {SensorModel::ROBOSENSE_EM4, "EM4", "Robosense EM4"},
    {SensorModel::ROBOSENSE_EMX, "EMX", "Robosense EMX"}
  };
}

std::vector<PacketChannelRequirement> RobosenseSensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.udp_destination_port = config.data_port;
  return {req};
}

std::vector<LiveTransportRequirement> RobosenseSensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.name = "robosense_udp_data";
  req.port = config.data_port;
  return {req};
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
