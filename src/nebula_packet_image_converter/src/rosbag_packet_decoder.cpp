// Copyright 2025 TIER IV, Inc.
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

/**
 * @file rosbag_packet_decoder.cpp
 * @brief Standalone tool to decode H.265 encoded images back to NebulaPackets.
 *
 * Usage:
 *   ros2 run nebula_packet_image_converter rosbag_packet_decoder \
 *     --input /path/to/encoded.bag \
 *     --output /path/to/decoded.bag \
 *     [--decoder libx265|hevc_cuvid]
 */

#include "nebula_packet_image_converter/packet_image_codec.hpp"

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <vector>

const std::vector<std::string> g_default_topics = {
  "/sensing/lidar/front_lower/pandar_packets_ffmpeg",
  "/sensing/lidar/front_upper/pandar_packets_ffmpeg",
  "/sensing/lidar/left_lower/pandar_packets_ffmpeg",
  "/sensing/lidar/left_upper/pandar_packets_ffmpeg",
  "/sensing/lidar/rear_lower/pandar_packets_ffmpeg",
  "/sensing/lidar/rear_upper/pandar_packets_ffmpeg",
  "/sensing/lidar/right_lower/pandar_packets_ffmpeg",
  "/sensing/lidar/right_upper/pandar_packets_ffmpeg",
};

void print_usage()
{
  std::cout << "Usage: rosbag_packet_decoder --input <bag> --output <bag> [options]\n";
  std::cout << "\nDecodes H.265 encoded images back to NebulaPackets in a rosbag.\n";
  std::cout << "\nOptions:\n";
  std::cout << "  --input    Input rosbag path (required)\n";
  std::cout << "  --output   Output rosbag path (required)\n";
  std::cout << "  --topics   Topics to decode (default: 8 pandar_packets_ffmpeg topics)\n";
  std::cout << "  --decoder  FFmpeg decoder name (default: hevc, or hevc_cuvid for GPU)\n";
}

int main(int argc, char ** argv)
{
  std::string input_bag;
  std::string output_bag;
  std::string decoder_name = "hevc";
  std::vector<std::string> topics = g_default_topics;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--input" && i + 1 < argc) {
      input_bag = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_bag = argv[++i];
    } else if (arg == "--decoder" && i + 1 < argc) {
      decoder_name = argv[++i];
    } else if (arg == "--topics") {
      topics.clear();
      while (i + 1 < argc && argv[i + 1][0] != '-') {
        topics.push_back(argv[++i]);
      }
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
  }

  if (input_bag.empty() || output_bag.empty()) {
    print_usage();
    return 1;
  }

  std::set<std::string> topics_to_decode(topics.begin(), topics.end());

  std::cout << "Input bag:  " << input_bag << "\n";
  std::cout << "Output bag: " << output_bag << "\n";
  std::cout << "Decoder:    " << decoder_name << "\n";
  std::cout << "Topics to decode:\n";
  for (const auto & t : topics_to_decode) {
    std::cout << "  - " << t << "\n";
  }

  try {
    rosbag2_cpp::Reader reader;
    reader.open(input_bag);

    rosbag2_cpp::Writer writer;
    writer.open(output_bag);

    // Per-topic decoders
    std::map<std::string, std::unique_ptr<ffmpeg_encoder_decoder::Decoder>> decoders;

    // Serializers
    rclcpp::Serialization<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> ffmpeg_serializer;
    rclcpp::Serialization<nebula_msgs::msg::NebulaPackets> packets_serializer;

    // Get topic metadata and register topics
    const auto bag_metadata = reader.get_metadata();
    for (const auto & topic_info : bag_metadata.topics_with_message_count) {
      const auto & topic = topic_info.topic_metadata;

      if (topics_to_decode.count(topic.name)) {
        // Replace ffmpeg topic with packets topic (remove _ffmpeg suffix)
        rosbag2_storage::TopicMetadata new_topic;
        std::string new_name = topic.name;
        // Remove _ffmpeg or _image suffix
        new_name = std::regex_replace(new_name, std::regex("_ffmpeg$"), "");
        new_name = std::regex_replace(new_name, std::regex("_image$"), "");
        new_topic.name = new_name;
        new_topic.type = "nebula_msgs/msg/NebulaPackets";
        new_topic.serialization_format = topic.serialization_format;
        writer.create_topic(new_topic);
        std::cout << "Created topic: " << new_topic.name << "\n";
      } else {
        // Copy other topics as-is
        writer.create_topic(topic);
      }
    }

    size_t decoded_count = 0;
    size_t copied_count = 0;

    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      if (topics_to_decode.count(bag_message->topic_name)) {
        // Deserialize ffmpeg packet
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        ffmpeg_image_transport_msgs::msg::FFMPEGPacket ffmpeg_pkt;
        ffmpeg_serializer.deserialize_message(&serialized_msg, &ffmpeg_pkt);

        try {
          // Initialize decoder for this topic if needed
          auto & decoder = decoders[bag_message->topic_name];

          // Get output topic name
          std::string output_topic = bag_message->topic_name;
          output_topic = std::regex_replace(output_topic, std::regex("_ffmpeg$"), "");
          output_topic = std::regex_replace(output_topic, std::regex("_image$"), "");

          if (!decoder) {
            decoder = std::make_unique<ffmpeg_encoder_decoder::Decoder>();

            // Callback when image is decoded
            auto callback = [&writer, &packets_serializer, output_topic,
                             timestamp = bag_message->time_stamp](
                              const sensor_msgs::msg::Image::ConstSharedPtr & img, bool /*isKeyFrame*/,
                              const std::string & /*avPixFormat*/) {
              // Convert image back to packets
              auto packets =
                nebula::packet_image_converter::PacketImageCodec::image_to_packets(*img);

              rclcpp::SerializedMessage pkt_serialized;
              packets_serializer.serialize_message(&packets, &pkt_serialized);

              auto out_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
              out_msg->topic_name = output_topic;
              out_msg->time_stamp = timestamp;
              out_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
              out_msg->serialized_data->buffer =
                pkt_serialized.get_rcl_serialized_message().buffer;
              out_msg->serialized_data->buffer_length =
                pkt_serialized.get_rcl_serialized_message().buffer_length;
              out_msg->serialized_data->buffer_capacity =
                pkt_serialized.get_rcl_serialized_message().buffer_capacity;
              out_msg->serialized_data->allocator =
                pkt_serialized.get_rcl_serialized_message().allocator;

              writer.write(out_msg);
            };

            if (!decoder->initialize(ffmpeg_pkt.encoding, callback, decoder_name)) {
              throw std::runtime_error("Failed to initialize decoder for " + ffmpeg_pkt.encoding);
            }
          }

          // Decode the packet
          rclcpp::Time stamp(ffmpeg_pkt.header.stamp);
          decoder->decodePacket(
            ffmpeg_pkt.encoding, ffmpeg_pkt.data.data(), ffmpeg_pkt.data.size(),
            ffmpeg_pkt.pts, ffmpeg_pkt.header.frame_id, stamp);

          decoded_count++;

          if (decoded_count % 100 == 0) {
            std::cout << "Decoded " << decoded_count << " messages...\r" << std::flush;
          }
        } catch (const std::exception & e) {
          std::cerr << "Warning: Failed to decode message on " << bag_message->topic_name << ": "
                    << e.what() << "\n";
        }
      } else {
        // Copy message as-is
        writer.write(bag_message);
        copied_count++;
      }
    }

    // Flush all decoders
    for (auto & [topic, decoder] : decoders) {
      if (decoder) {
        decoder->flush();
      }
    }

    std::cout << "\nDone!\n";
    std::cout << "Decoded: " << decoded_count << " messages\n";
    std::cout << "Copied:  " << copied_count << " messages\n";

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
