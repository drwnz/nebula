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

#include "continental_decoder_agnostic.hpp"

#include <nebula_continental_decoders/decoders/continental_ars548_decoder.hpp>
#include <nebula_continental_decoders/decoders/continental_srr520_decoder.hpp>
#include <continental_msgs/msg/continental_ars548_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>

#include <cmath>

namespace nebula::drivers {

// ARS548 Impl
struct AgnosticARS548Decoder::Impl {
    std::unique_ptr<continental_ars548::ContinentalARS548Decoder> decoder;
    std::function<void(std::shared_ptr<RadarDetectionList>)> callback;
};

AgnosticARS548Decoder::AgnosticARS548Decoder(const std::shared_ptr<const continental_ars548::ContinentalARS548SensorConfiguration>& config)
: impl_(std::make_unique<Impl>())
{
    impl_->decoder = std::make_unique<continental_ars548::ContinentalARS548Decoder>(config);
    impl_->decoder->register_detection_list_callback(
        [this](std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg) {
            if (impl_->callback) {
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
                impl_->callback(list);
            }
        });
}

AgnosticARS548Decoder::~AgnosticARS548Decoder() = default;

bool AgnosticARS548Decoder::process_packet(const std::vector<uint8_t>& packet_data, uint64_t timestamp_ns)
{
    auto packet_msg = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    packet_msg->data = packet_data;
    packet_msg->stamp.sec = timestamp_ns / 1000000000ULL;
    packet_msg->stamp.nanosec = timestamp_ns % 1000000000ULL;
    return impl_->decoder->process_packet(std::move(packet_msg));
}

void AgnosticARS548Decoder::set_detection_callback(std::function<void(std::shared_ptr<RadarDetectionList>)> cb)
{
    impl_->callback = cb;
}

// SRR520 Impl
struct AgnosticSRR520Decoder::Impl {
    std::unique_ptr<continental_srr520::ContinentalSRR520Decoder> decoder;
    std::function<void(std::shared_ptr<RadarDetectionList>)> callback;
};

AgnosticSRR520Decoder::AgnosticSRR520Decoder(const std::shared_ptr<const continental_srr520::ContinentalSRR520SensorConfiguration>& config)
: impl_(std::make_unique<Impl>())
{
    impl_->decoder = std::make_unique<continental_srr520::ContinentalSRR520Decoder>(config);
    
    auto on_det = [this](std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg) {
        if (impl_->callback) {
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
            impl_->callback(list);
        }
    };

    impl_->decoder->register_near_detection_list_callback(on_det);
    impl_->decoder->register_hrr_detection_list_callback(on_det);
}

AgnosticSRR520Decoder::~AgnosticSRR520Decoder() = default;

bool AgnosticSRR520Decoder::process_packet(const std::vector<uint8_t>& packet_data, uint64_t timestamp_ns)
{
    auto packet_msg = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    packet_msg->data = packet_data;
    packet_msg->stamp.sec = timestamp_ns / 1000000000ULL;
    packet_msg->stamp.nanosec = timestamp_ns % 1000000000ULL;
    return impl_->decoder->process_packet(std::move(packet_msg));
}

void AgnosticSRR520Decoder::set_detection_callback(std::function<void(std::shared_ptr<RadarDetectionList>)> cb)
{
    impl_->callback = cb;
}

}  // namespace nebula::drivers
