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

#include <nebula_continental_decoders/continental_plugin.hpp>

#include <iostream>

namespace nebula::drivers
{

// ARS548 Runtime
void ContinentalARS548SensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  auto c_config = std::make_shared<continental_ars548::ContinentalARS548SensorConfiguration>();
  c_config->sensor_model = config.sensor_model;
  c_config->host_ip = config.host_ip;
  c_config->sensor_ip = config.sensor_ip;
  c_config->data_port = config.data_port;
  c_config->frame_id = config.frame_id;
  c_config->base_frame = config.frame_id;
  c_config->object_frame = config.frame_id;
  c_config->use_sensor_time = true;

  decoder_ = std::make_unique<continental_ars548::ContinentalARS548Decoder>(c_config);
  decoder_->register_detection_list_callback(
    std::bind(&ContinentalARS548SensorDecoderRuntime::on_detections, this, std::placeholders::_1));
}

SensorPacketResult ContinentalARS548SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!decoder_) return SensorPacketResult::Error;
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

void ContinentalARS548SensorDecoderRuntime::on_detections(std::shared_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)
{
  progress_.output_count++;
  if (output_callback_) {
    SensorDecodedOutput output;
    output.kind = SensorOutputKind::RadarDetections;
    output.timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;
    output.sensor_id = "continental_ars548";
    output.payload = msg;
    output_callback_(output);
  }
}

// SRR520 Runtime
void ContinentalSRR520SensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  auto s_config = std::make_shared<continental_srr520::ContinentalSRR520SensorConfiguration>();
  s_config->sensor_model = config.sensor_model;
  s_config->frame_id = config.frame_id;
  s_config->base_frame = config.frame_id;

  decoder_ = std::make_unique<continental_srr520::ContinentalSRR520Decoder>(s_config);
  decoder_->register_near_detection_list_callback(
    std::bind(&ContinentalSRR520SensorDecoderRuntime::on_detections, this, std::placeholders::_1));
  decoder_->register_hrr_detection_list_callback(
    std::bind(&ContinentalSRR520SensorDecoderRuntime::on_detections, this, std::placeholders::_1));
}

SensorPacketResult ContinentalSRR520SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!decoder_) return SensorPacketResult::Error;
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

void ContinentalSRR520SensorDecoderRuntime::on_detections(std::shared_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)
{
  progress_.output_count++;
  if (output_callback_) {
    SensorDecodedOutput output;
    output.kind = SensorOutputKind::RadarDetections;
    output.timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;
    output.sensor_id = "continental_srr520";
    output.payload = msg;
    output_callback_(output);
  }
}

// Plugin
SensorPluginMetadata ContinentalSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "continental";
  md.package_name = "nebula_continental_decoders";
  md.library_path = "libnebula_continental_decoders_plugin.so";
  md.supported_models = {SensorModel::CONTINENTAL_ARS548, SensorModel::CONTINENTAL_SRR520};
  return md;
}

std::vector<SensorModelInfo> ContinentalSensorPlugin::supported_models() const
{
  return {
    {SensorModel::CONTINENTAL_ARS548, "ARS548", "Continental ARS548 Radar"},
    {SensorModel::CONTINENTAL_SRR520, "SRR520", "Continental SRR520 Radar"}
  };
}

std::vector<PacketChannelRequirement> ContinentalSensorPlugin::packet_requirements(const SensorConfiguration & config) const
{
  current_model_ = config.sensor_model;
  PacketChannelRequirement req;
  req.channel = SensorPacketChannel::Radar;
  req.required = true;
  if (config.sensor_model == SensorModel::CONTINENTAL_ARS548) {
      req.transport = SensorTransportKind::UDP;
      req.udp_destination_port = config.data_port;
  } else {
      req.transport = SensorTransportKind::CAN;
  }
  return {req};
}

std::vector<LiveTransportRequirement> ContinentalSensorPlugin::live_transport_requirements(const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.channel = SensorPacketChannel::Radar;
  req.required = true;
  req.name = "continental_radar";
  if (config.sensor_model == SensorModel::CONTINENTAL_ARS548) {
      req.transport = SensorTransportKind::UDP;
      req.port = config.data_port;
  } else {
      req.transport = SensorTransportKind::CAN;
  }
  return {req};
}

std::unique_ptr<SensorDecoderRuntime> ContinentalSensorPlugin::create_decoder_runtime() const
{
  if (current_model_ == SensorModel::CONTINENTAL_ARS548) {
      return std::make_unique<ContinentalARS548SensorDecoderRuntime>();
  } else {
      return std::make_unique<ContinentalSRR520SensorDecoderRuntime>();
  }
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::ContinentalSensorPlugin();
}
}
