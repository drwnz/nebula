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

#include <nebula_continental_decoders/continental_ars548_plugin.hpp>

#include <iostream>

namespace nebula::drivers
{

ContinentalARS548SensorDecoderRuntime::ContinentalARS548SensorDecoderRuntime()
{
}

void ContinentalARS548SensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  config_ = config;
  
  auto c_config = std::make_shared<continental_ars548::ContinentalARS548SensorConfiguration>();
  c_config->sensor_model = config.sensor_model;
  c_config->host_ip = config.host_ip;
  c_config->sensor_ip = config.sensor_ip;
  c_config->data_port = config.data_port;
  c_config->frame_id = config.frame_id;
  c_config->base_frame = config.frame_id;
  c_config->object_frame = config.frame_id;
  
  // Defaults
  c_config->configuration_host_port = 0;
  c_config->configuration_sensor_port = 0;
  c_config->use_sensor_time = true;

  decoder_ = std::make_unique<continental_ars548::ContinentalARS548Decoder>(c_config);
  
  decoder_->register_detection_list_callback(
    std::bind(&ContinentalARS548SensorDecoderRuntime::on_detections, this, std::placeholders::_1));
  decoder_->register_object_list_callback(
    std::bind(&ContinentalARS548SensorDecoderRuntime::on_objects, this, std::placeholders::_1));
}

void ContinentalARS548SensorDecoderRuntime::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
}

void ContinentalARS548SensorDecoderRuntime::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void ContinentalARS548SensorDecoderRuntime::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
}

SensorPacketResult ContinentalARS548SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!decoder_) {
    return SensorPacketResult::Error;
  }

  progress_.processed_packets++;

  auto packet_msg = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  packet_msg->data = packet.payload;
  packet_msg->stamp.sec = packet.timestamp_ns / 1000000000ULL;
  packet_msg->stamp.nanosec = packet.timestamp_ns % 1000000000ULL;
  
  if (decoder_->process_packet(std::move(packet_msg))) {
    progress_.matched_packets++;
    progress_.decoded_packets++;
    if (progress_callback_) progress_callback_(progress_);
    return SensorPacketResult::Success;
  }

  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

void ContinentalARS548SensorDecoderRuntime::flush()
{
}

void ContinentalARS548SensorDecoderRuntime::on_detections(std::shared_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)
{
  progress_.output_count++;
  if (output_callback_) {
    SensorDecodedOutput output;
    output.kind = SensorOutputKind::RadarDetections;
    output.timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;
    output.sensor_id = "continental_ars548";
    output.payload = msg; // shared_ptr is copyable
    output_callback_(output);
  }
}

void ContinentalARS548SensorDecoderRuntime::on_objects([[maybe_unused]] std::shared_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg)
{
    // TODO: implement when needed
}

SensorPluginMetadata ContinentalARS548SensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "continental";
  md.package_name = "nebula_continental_decoders";
  md.library_path = "libnebula_continental_decoders_plugin.so";
  md.supported_models = {SensorModel::CONTINENTAL_ARS548};
  return md;
}

std::vector<SensorModelInfo> ContinentalARS548SensorPlugin::supported_models() const
{
  return {{SensorModel::CONTINENTAL_ARS548, "ARS548", "Continental ARS548 Radar"}};
}

std::vector<PacketChannelRequirement> ContinentalARS548SensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Radar;
  req.required = true;
  req.udp_destination_port = config.data_port;
  return {req};
}

std::vector<LiveTransportRequirement> ContinentalARS548SensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Radar;
  req.required = true;
  req.name = "continental_udp_radar";
  req.port = config.data_port;
  return {req};
}

std::unique_ptr<SensorDecoderRuntime> ContinentalARS548SensorPlugin::create_decoder_runtime() const
{
  return std::make_unique<ContinentalARS548SensorDecoderRuntime>();
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::ContinentalARS548SensorPlugin();
}
}
