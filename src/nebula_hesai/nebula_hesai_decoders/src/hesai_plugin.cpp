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

#include <nebula_hesai_decoders/hesai_plugin.hpp>
#include <nebula_core_common/loggers/console_logger.hpp>

#include <iostream>

namespace nebula::drivers
{

HesaiSensorDecoderRuntime::HesaiSensorDecoderRuntime()
{
}

void HesaiSensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  config_ = config;
  
  auto h_config = std::make_shared<HesaiSensorConfiguration>();
  h_config->sensor_model = config.sensor_model;
  h_config->host_ip = config.host_ip;
  h_config->sensor_ip = config.sensor_ip;
  h_config->data_port = config.data_port;
  h_config->frame_id = config.frame_id;
  
  // Defaults
  h_config->rotation_speed = 600;
  h_config->cloud_min_angle = 0;
  h_config->cloud_max_angle = 360;
  h_config->calibration_download_enabled = false;

  std::shared_ptr<HesaiCalibrationConfigurationBase> c_config;
  if (config.sensor_model == SensorModel::HESAI_PANDARAT128) {
      c_config = std::make_shared<HesaiCorrection>();
  } else if (config.sensor_model == SensorModel::HESAI_FTX140 || config.sensor_model == SensorModel::HESAI_FTX180) {
      c_config = std::make_shared<HesaiCorrectionFTX>();
  } else {
      c_config = std::make_shared<HesaiCalibrationConfiguration>();
  }
  
  auto logger = std::make_shared<loggers::ConsoleLogger>("HesaiPlugin");

  driver_ = std::make_unique<HesaiDriver>(
    h_config, c_config, logger,
    std::bind(&HesaiSensorDecoderRuntime::on_pointcloud, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void HesaiSensorDecoderRuntime::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
}

void HesaiSensorDecoderRuntime::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void HesaiSensorDecoderRuntime::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
}

SensorPacketResult HesaiSensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!driver_) {
    return SensorPacketResult::Error;
  }

  progress_.processed_packets++;

  if (packet.channel == SensorPacketChannel::Data || packet.channel == SensorPacketChannel::Unknown) {
    progress_.matched_packets++;
    
    auto result = driver_->parse_cloud_packet(packet.payload);
    
    if (result.metadata_or_error.has_value()) {
        progress_.decoded_packets++;
    } else {
        progress_.error_count++;
        // TODO: error callback
    }

    if (progress_callback_) progress_callback_(progress_);
    return SensorPacketResult::Success;
  }

  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

void HesaiSensorDecoderRuntime::flush()
{
}

void HesaiSensorDecoderRuntime::on_pointcloud(const NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  progress_.output_count++;
  if (output_callback_) {
    SensorDecodedOutput output;
    output.kind = SensorOutputKind::PointCloud;
    output.timestamp_ns = static_cast<uint64_t>(timestamp_s * 1e9);
    output.sensor_id = "hesai";
    output.payload = pointcloud;
    output_callback_(output);
  }
}

SensorPluginMetadata HesaiSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "hesai";
  md.package_name = "nebula_hesai_decoders";
  md.library_path = "libnebula_hesai_decoders_plugin.so";
  md.supported_models = {
    SensorModel::HESAI_PANDAR64,
    SensorModel::HESAI_PANDAR40P,
    SensorModel::HESAI_PANDAR40M,
    SensorModel::HESAI_PANDARQT64,
    SensorModel::HESAI_PANDARQT128,
    SensorModel::HESAI_PANDARXT16,
    SensorModel::HESAI_PANDARXT32,
    SensorModel::HESAI_PANDARXT32M,
    SensorModel::HESAI_PANDARAT128,
    SensorModel::HESAI_FTX140,
    SensorModel::HESAI_FTX180,
    SensorModel::HESAI_PANDAR128_E3X,
    SensorModel::HESAI_PANDAR128_E4X
  };
  return md;
}

std::vector<SensorModelInfo> HesaiSensorPlugin::supported_models() const
{
  return {
    {SensorModel::HESAI_PANDAR64, "Pandar64", "Hesai Pandar64"},
    {SensorModel::HESAI_PANDAR40P, "Pandar40P", "Hesai Pandar40P"},
    {SensorModel::HESAI_PANDAR40M, "Pandar40M", "Hesai Pandar40M"},
    {SensorModel::HESAI_PANDARQT64, "PandarQT64", "Hesai PandarQT64"},
    {SensorModel::HESAI_PANDARQT128, "PandarQT128", "Hesai PandarQT128"},
    {SensorModel::HESAI_PANDARXT16, "PandarXT16", "Hesai PandarXT16"},
    {SensorModel::HESAI_PANDARXT32, "PandarXT32", "Hesai PandarXT32"},
    {SensorModel::HESAI_PANDARXT32M, "PandarXT32M", "Hesai PandarXT32M"},
    {SensorModel::HESAI_PANDARAT128, "PandarAT128", "Hesai PandarAT128"},
    {SensorModel::HESAI_FTX140, "FTX140", "Hesai FTX140"},
    {SensorModel::HESAI_FTX180, "FTX180", "Hesai FTX180"},
    {SensorModel::HESAI_PANDAR128_E3X, "Pandar128_E3X", "Hesai Pandar128 E3X"},
    {SensorModel::HESAI_PANDAR128_E4X, "Pandar128_E4X", "Hesai Pandar128 E4X"}
  };
}

std::vector<PacketChannelRequirement> HesaiSensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.udp_destination_port = config.data_port;
  return {req};
}

std::vector<LiveTransportRequirement> HesaiSensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.name = "hesai_udp_data";
  req.port = config.data_port;
  return {req};
}

std::unique_ptr<SensorDecoderRuntime> HesaiSensorPlugin::create_decoder_runtime() const
{
  return std::make_unique<HesaiSensorDecoderRuntime>();
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::HesaiSensorPlugin();
}
}
