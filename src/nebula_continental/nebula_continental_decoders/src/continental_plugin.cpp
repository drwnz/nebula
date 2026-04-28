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

// Include the agnostic wrapper, completely hiding ROS message headers from this compilation unit.
#include "continental_decoder_agnostic.hpp"

#include <iostream>
#include <cmath>

namespace nebula::drivers
{

// ARS548 Runtime
struct ContinentalARS548SensorDecoderRuntime::Impl
{
    std::unique_ptr<AgnosticARS548Decoder> decoder;
};

ContinentalARS548SensorDecoderRuntime::ContinentalARS548SensorDecoderRuntime()
: impl_(std::make_unique<Impl>())
{
}

ContinentalARS548SensorDecoderRuntime::~ContinentalARS548SensorDecoderRuntime() = default;

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

  if (config.extra_params.count("multicast_ip")) c_config->multicast_ip = config.extra_params.at("multicast_ip");

  impl_->decoder = std::make_unique<AgnosticARS548Decoder>(c_config);
  
  impl_->decoder->set_detection_callback([this](std::shared_ptr<RadarDetectionList> list) {
      progress_.output_count++;
      if (output_callback_) {
        SensorDecodedOutput output;
        output.kind = SensorOutputKind::RadarDetections;
        output.timestamp_ns = list->timestamp_ns;
        output.sensor_id = "continental_ars548";
        output.payload = list;
        output_callback_(output);
      }
  });
}

SensorPacketResult ContinentalARS548SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!impl_->decoder) return SensorPacketResult::Error;
  progress_.processed_packets++;
  
  if (impl_->decoder->process_packet(packet.payload, packet.timestamp_ns)) {
    progress_.matched_packets++;
    progress_.decoded_packets++;
    if (progress_callback_) progress_callback_(progress_);
    return SensorPacketResult::Success;
  }
  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

// SRR520 Runtime
struct ContinentalSRR520SensorDecoderRuntime::Impl
{
    std::unique_ptr<AgnosticSRR520Decoder> decoder;
};

ContinentalSRR520SensorDecoderRuntime::ContinentalSRR520SensorDecoderRuntime()
: impl_(std::make_unique<Impl>())
{
}

ContinentalSRR520SensorDecoderRuntime::~ContinentalSRR520SensorDecoderRuntime() = default;

void ContinentalSRR520SensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  auto s_config = std::make_shared<continental_srr520::ContinentalSRR520SensorConfiguration>();
  s_config->sensor_model = config.sensor_model;
  s_config->frame_id = config.frame_id;
  s_config->base_frame = config.extra_params.count("base_frame") ? config.extra_params.at("base_frame") : "base_link";

  impl_->decoder = std::make_unique<AgnosticSRR520Decoder>(s_config);
  
  impl_->decoder->set_detection_callback([this](std::shared_ptr<RadarDetectionList> list) {
      progress_.output_count++;
      if (output_callback_) {
        SensorDecodedOutput output;
        output.kind = SensorOutputKind::RadarDetections;
        output.timestamp_ns = list->timestamp_ns;
        output.sensor_id = "continental_srr520";
        output.payload = list;
        output_callback_(output);
      }
  });
}

SensorPacketResult ContinentalSRR520SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!impl_->decoder) return SensorPacketResult::Error;
  progress_.processed_packets++;
  
  if (impl_->decoder->process_packet(packet.payload, packet.timestamp_ns)) {
      progress_.matched_packets++;
      progress_.decoded_packets++;
      if (progress_callback_) progress_callback_(progress_);
      return SensorPacketResult::Success;
  }

  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

// Plugin
SensorPluginMetadata ContinentalSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "continental";
  md.package_name = "nebula_continental_decoders";
  md.library_path = "libnebula_continental_decoders_plugin.so";
  md.factory_symbol = "create_nebula_sensor_plugin";
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
