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

#include <nebula_seyond_decoders/seyond_plugin.hpp>

#include <iostream>

namespace nebula::drivers
{

SeyondSensorDecoderRuntime::SeyondSensorDecoderRuntime()
{
}

void SeyondSensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  config_ = config;
  
  SeyondSensorConfiguration s_config;
  s_config.connection.host_ip = config.host_ip;
  s_config.connection.sensor_ip = config.sensor_ip;
  s_config.connection.udp_port = config.data_port;
  s_config.frame_id = config.frame_id;
  
  // Use expanded config fields
  s_config.fov.azimuth.start = config.fov.azimuth.start;
  s_config.fov.azimuth.end = config.fov.azimuth.end;
  s_config.fov.elevation.start = config.fov.elevation.start;
  s_config.fov.elevation.end = config.fov.elevation.end;

  switch (config.sensor_model) {
      case SensorModel::SEYOND_FALCON_K: s_config.sensor_model = SeyondSensorModel::FALCON_K; break;
      case SensorModel::SEYOND_ROBIN_W: s_config.sensor_model = SeyondSensorModel::ROBIN_W; break;
      case SensorModel::SEYOND_ROBIN_E1X: s_config.sensor_model = SeyondSensorModel::ROBIN_E1X; break;
      case SensorModel::SEYOND_HUMMINGBIRD_D1: s_config.sensor_model = SeyondSensorModel::HUMMINGBIRD_D1; break;
      default: throw std::runtime_error("Unsupported Seyond model");
  }

  decoder_ = std::make_unique<SeyondDecoder>(
    s_config,
    std::bind(&SeyondSensorDecoderRuntime::on_pointcloud, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void SeyondSensorDecoderRuntime::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
}

void SeyondSensorDecoderRuntime::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void SeyondSensorDecoderRuntime::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
}

SensorPacketResult SeyondSensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!decoder_) {
    return SensorPacketResult::Error;
  }

  progress_.processed_packets++;

  if (packet.channel == SensorPacketChannel::Data || packet.channel == SensorPacketChannel::Unknown) {
    progress_.matched_packets++;
    
    [[maybe_unused]] auto result = decoder_->unpack(packet.payload);
    
    progress_.decoded_packets++;

    if (progress_callback_) progress_callback_(progress_);
    return SensorPacketResult::Success;
  }

  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

void SeyondSensorDecoderRuntime::flush()
{
}

void SeyondSensorDecoderRuntime::on_pointcloud(NebulaPointCloudPtr pointcloud, uint64_t timestamp_ns)
{
  progress_.output_count++;
  if (output_callback_) {
    SensorDecodedOutput output;
    output.kind = SensorOutputKind::PointCloud;
    output.timestamp_ns = timestamp_ns;
    output.sensor_id = "seyond";
    output.payload = pointcloud;
    output_callback_(output);
  }
}

SensorPluginMetadata SeyondSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "seyond";
  md.package_name = "nebula_seyond_decoders";
  md.library_path = "libnebula_seyond_decoders_plugin.so";
  md.supported_models = {
    SensorModel::SEYOND_FALCON_K,
    SensorModel::SEYOND_ROBIN_W,
    SensorModel::SEYOND_ROBIN_E1X,
    SensorModel::SEYOND_HUMMINGBIRD_D1
  };
  return md;
}

std::vector<SensorModelInfo> SeyondSensorPlugin::supported_models() const
{
  return {
    {SensorModel::SEYOND_FALCON_K, "FalconK", "Seyond Falcon K"},
    {SensorModel::SEYOND_ROBIN_W, "RobinW", "Seyond Robin W"},
    {SensorModel::SEYOND_ROBIN_E1X, "RobinE1X", "Seyond Robin E1X"},
    {SensorModel::SEYOND_HUMMINGBIRD_D1, "HummingbirdD1", "Seyond Hummingbird D1"}
  };
}

std::vector<PacketChannelRequirement> SeyondSensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.udp_destination_port = config.data_port;
  return {req};
}

std::vector<LiveTransportRequirement> SeyondSensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.name = "seyond_udp_data";
  req.port = config.data_port;
  return {req};
}

std::unique_ptr<SensorDecoderRuntime> SeyondSensorPlugin::create_decoder_runtime() const
{
  return std::make_unique<SeyondSensorDecoderRuntime>();
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::SeyondSensorPlugin();
}
}
