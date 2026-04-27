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

#include <nebula_velodyne_decoders/velodyne_plugin.hpp>

#include <iostream>

namespace nebula::drivers
{

VelodyneSensorDecoderRuntime::VelodyneSensorDecoderRuntime()
{
}

void VelodyneSensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  config_ = config;
  
  auto v_config = std::make_shared<VelodyneSensorConfiguration>();
  v_config->sensor_model = config.sensor_model;
  v_config->host_ip = config.host_ip;
  v_config->sensor_ip = config.sensor_ip;
  v_config->data_port = config.data_port;
  v_config->frame_id = config.frame_id;
  
  // Default values for Velodyne-specific fields if not provided
  v_config->rotation_speed = 600;
  v_config->cloud_min_angle = 0;
  v_config->cloud_max_angle = 360;

  auto c_config = std::make_shared<VelodyneCalibrationConfiguration>();
  // In a real scenario, we'd load calibration from a path in config or a default path
  
  driver_ = std::make_unique<VelodyneDriver>(v_config, c_config);
}

void VelodyneSensorDecoderRuntime::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
}

void VelodyneSensorDecoderRuntime::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void VelodyneSensorDecoderRuntime::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
}

SensorPacketResult VelodyneSensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!driver_) {
    return SensorPacketResult::Error;
  }

  progress_.processed_packets++;

  if (packet.channel == SensorPacketChannel::Data || packet.channel == SensorPacketChannel::Unknown) {
    progress_.matched_packets++;
    
    double timestamp_s = static_cast<double>(packet.timestamp_ns) * 1e-9;
    auto [pointcloud, scan_timestamp] = driver_->parse_cloud_packet(packet.payload, timestamp_s);
    
    progress_.decoded_packets++;

    if (pointcloud) {
      progress_.output_count++;
      if (output_callback_) {
        SensorDecodedOutput output;
        output.kind = SensorOutputKind::PointCloud;
        output.timestamp_ns = static_cast<uint64_t>(scan_timestamp * 1e9);
        output.sensor_id = "velodyne";
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

void VelodyneSensorDecoderRuntime::flush()
{
}

SensorPluginMetadata VelodyneSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "velodyne";
  md.package_name = "nebula_velodyne_decoders";
  md.library_path = "libnebula_velodyne_decoders_plugin.so";
  md.supported_models = {
    SensorModel::VELODYNE_VLS128,
    SensorModel::VELODYNE_HDL64,
    SensorModel::VELODYNE_VLP32,
    SensorModel::VELODYNE_VLP32MR,
    SensorModel::VELODYNE_HDL32,
    SensorModel::VELODYNE_VLP16
  };
  return md;
}

std::vector<SensorModelInfo> VelodyneSensorPlugin::supported_models() const
{
  return {
    {SensorModel::VELODYNE_VLS128, "VLS128", "Velodyne VLS-128"},
    {SensorModel::VELODYNE_HDL64, "HDL64", "Velodyne HDL-64"},
    {SensorModel::VELODYNE_VLP32, "VLP32", "Velodyne VLP-32"},
    {SensorModel::VELODYNE_VLP32MR, "VLP32MR", "Velodyne VLP-32MR"},
    {SensorModel::VELODYNE_HDL32, "HDL32", "Velodyne HDL-32"},
    {SensorModel::VELODYNE_VLP16, "VLP16", "Velodyne VLP-16"}
  };
}

std::vector<PacketChannelRequirement> VelodyneSensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.udp_destination_port = config.data_port;
  return {req};
}

std::vector<LiveTransportRequirement> VelodyneSensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.name = "velodyne_udp_data";
  req.port = config.data_port;
  return {req};
}

std::unique_ptr<SensorDecoderRuntime> VelodyneSensorPlugin::create_decoder_runtime() const
{
  return std::make_unique<VelodyneSensorDecoderRuntime>();
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::VelodyneSensorPlugin();
}
}
