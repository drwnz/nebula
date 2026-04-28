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

#include <nebula_continental_decoders/decoders/continental_ars548_decoder.hpp>
#include <nebula_continental_decoders/decoders/continental_srr520_decoder.hpp>

#include <continental_msgs/msg/continental_ars548_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>

#include <iostream>
#include <cmath>

using namespace nebula::drivers::continental_ars548;
using namespace nebula::drivers::continental_srr520;

namespace nebula::drivers
{

// ARS548 Runtime
struct ContinentalARS548SensorDecoderRuntime::Impl
{
    std::unique_ptr<continental_ars548::ContinentalARS548Decoder> decoder;
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

  impl_->decoder = std::make_unique<continental_ars548::ContinentalARS548Decoder>(c_config);
  
  auto on_det = [this](std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg) {
      progress_.output_count++;
      if (output_callback_) {
        auto list = std::make_shared<RadarDetectionList>();
        list->timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;
        list->frame_id = msg->header.frame_id;
        for (const auto & d : msg->detections) {
            RadarDetection det;
            det.range = d.range;
            det.azimuth = d.azimuth_angle;
            det.elevation = d.elevation_angle;
            det.x = d.range * std::cos(d.elevation_angle) * std::cos(d.azimuth_angle);
            det.y = d.range * std::cos(d.elevation_angle) * std::sin(d.azimuth_angle);
            det.z = d.range * std::sin(d.elevation_angle);
            det.range_rate = d.range_rate;
            det.rcs = d.rcs;
            det.id = d.measurement_id;
            det.classification = d.classification;
            list->detections.push_back(det);
        }
        
        SensorDecodedOutput output;
        output.kind = SensorOutputKind::RadarDetections;
        output.timestamp_ns = list->timestamp_ns;
        output.sensor_id = "continental_ars548";
        output.payload = list;
        output_callback_(output);
      }
  };

  impl_->decoder->register_detection_list_callback(on_det);
}

SensorPacketResult ContinentalARS548SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!impl_->decoder) return SensorPacketResult::Error;
  progress_.processed_packets++;
  
  auto packet_msg = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  packet_msg->data = packet.payload;
  packet_msg->stamp.sec = packet.timestamp_ns / 1000000000ULL;
  packet_msg->stamp.nanosec = packet.timestamp_ns % 1000000000ULL;
  
  if (impl_->decoder->process_packet(std::move(packet_msg))) {
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
    std::unique_ptr<continental_srr520::ContinentalSRR520Decoder> decoder;
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

  impl_->decoder = std::make_unique<continental_srr520::ContinentalSRR520Decoder>(s_config);
  
  auto on_det = [this](std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg) {
      progress_.output_count++;
      if (output_callback_) {
        auto list = std::make_shared<RadarDetectionList>();
        list->timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;
        list->frame_id = msg->header.frame_id;
        for (const auto & d : msg->detections) {
            RadarDetection det;
            det.range = d.range;
            det.azimuth = d.azimuth_angle;
            det.elevation = 0.0f;
            det.x = d.range * std::cos(d.azimuth_angle);
            det.y = d.range * std::sin(d.azimuth_angle);
            det.z = 0.0f;
            det.range_rate = d.range_rate;
            det.rcs = d.rcs;
            det.id = 0;
            det.classification = 0;
            list->detections.push_back(det);
        }

        SensorDecodedOutput output;
        output.kind = SensorOutputKind::RadarDetections;
        output.timestamp_ns = list->timestamp_ns;
        output.sensor_id = "continental_srr520";
        output.payload = list;
        output_callback_(output);
      }
  };

  impl_->decoder->register_near_detection_list_callback(on_det);
  impl_->decoder->register_hrr_detection_list_callback(on_det);
}

SensorPacketResult ContinentalSRR520SensorDecoderRuntime::process_packet(const SensorPacket & packet)
{
  if (!impl_->decoder) return SensorPacketResult::Error;
  progress_.processed_packets++;
  
  auto packet_msg = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  packet_msg->data = packet.payload;
  packet_msg->stamp.sec = packet.timestamp_ns / 1000000000ULL;
  packet_msg->stamp.nanosec = packet.timestamp_ns % 1000000000ULL;

  if (impl_->decoder->process_packet(std::move(packet_msg))) {
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
