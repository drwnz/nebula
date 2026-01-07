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
 * @file rosbag_packet_encoder.cpp
 * @brief Standalone tool to convert NebulaPackets in a rosbag to H.265 encoded images.
 *
 * Usage:
 *   ros2 run nebula_packet_image_converter rosbag_packet_encoder \
 *     --input /path/to/input.bag \
 *     --output /path/to/output.bag \
 *     [--topics /topic1 /topic2 ...] \
 *     [--encoder libx265|hevc_nvenc]
 */

#include "nebula_packet_image_converter/packet_image_codec.hpp"

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

const std::vector<std::string> g_default_topics = {
  "/sensing/lidar/front_lower/pandar_packets",
  "/sensing/lidar/front_upper/pandar_packets",
  "/sensing/lidar/left_lower/pandar_packets",
  "/sensing/lidar/left_upper/pandar_packets",
  "/sensing/lidar/rear_lower/pandar_packets",
  "/sensing/lidar/rear_upper/pandar_packets",
  "/sensing/lidar/right_lower/pandar_packets",
  "/sensing/lidar/right_upper/pandar_packets",
};

void print_usage()
{
  std::cout << "Usage: rosbag_packet_encoder --input <bag> --output <bag> [options]\n";
  std::cout << "\nConverts NebulaPackets messages to H.265 encoded images in a rosbag.\n";
  std::cout << "\nOptions:\n";
  std::cout << "  --input    Input rosbag path (required)\n";
  std::cout << "  --output   Output rosbag path (required)\n";
  std::cout << "  --topics   Topics to convert (default: 8 pandar_packets topics)\n";
  std::cout << "  --encoder  FFmpeg encoder name (default: libx265, or hevc_nvenc for GPU)\n";
  std::cout << "  --raw      Output raw images instead of encoded (for debugging)\n";
}

int main(int argc, char ** argv)
{
  std::string input_bag;
  std::string output_bag;
  std::string encoder_name = "libx264";
  std::vector<std::string> topics = g_default_topics;
  bool raw_output = false;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--input" && i + 1 < argc) {
      input_bag = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_bag = argv[++i];
    } else if (arg == "--encoder" && i + 1 < argc) {
      encoder_name = argv[++i];
    } else if (arg == "--raw") {
      raw_output = true;
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

  std::set<std::string> topics_to_convert(topics.begin(), topics.end());

  std::cout << "Input bag:  " << input_bag << "\n";
  std::cout << "Output bag: " << output_bag << "\n";
  std::cout << "Encoder:    " << (raw_output ? "(raw - no encoding)" : encoder_name) << "\n";
  std::cout << "Topics to convert:\n";
  for (const auto & t : topics_to_convert) {
    std::cout << "  - " << t << "\n";
  }

  try {
    rosbag2_cpp::Reader reader;
    reader.open(input_bag);

    rosbag2_cpp::Writer writer;
    writer.open(output_bag);

    // Per-topic encoders (each topic needs its own encoder for state)
    std::map<std::string, std::unique_ptr<ffmpeg_encoder_decoder::Encoder>> encoders;

    // Serializers
    rclcpp::Serialization<pandar_msgs::msg::PandarScan> packets_serializer;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    rclcpp::Serialization<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> ffmpeg_serializer;

    // Get topic metadata and register topics
    const auto bag_metadata = reader.get_metadata();
    for (const auto & topic_info : bag_metadata.topics_with_message_count) {
      const auto & topic = topic_info.topic_metadata;

      if (topics_to_convert.count(topic.name)) {
        // Replace packet topic with encoded image topic
        rosbag2_storage::TopicMetadata new_topic;
        if (raw_output) {
          new_topic.name = topic.name + "_image";
          new_topic.type = "sensor_msgs/msg/Image";
        } else {
          new_topic.name = topic.name + "_ffmpeg";
          new_topic.type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
        }
        new_topic.serialization_format = topic.serialization_format;
        writer.create_topic(new_topic);
        std::cout << "Created topic: " << new_topic.name << "\n";
      } else {
        // Copy other topics as-is
        writer.create_topic(topic);
      }
    }

    size_t converted_count = 0;
    size_t copied_count = 0;

    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      if (topics_to_convert.count(bag_message->topic_name)) {
        // Deserialize PandarScan
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pandar_msgs::msg::PandarScan scan;
        packets_serializer.deserialize_message(&serialized_msg, &scan);

        // Convert to image
        try {
          auto image = nebula::packet_image_converter::PacketImageCodec::pandar_scan_to_image(scan);

          if (raw_output) {
            // Write raw image
            rclcpp::SerializedMessage image_serialized;
            image_serializer.serialize_message(&image, &image_serialized);

            auto out_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            out_msg->topic_name = bag_message->topic_name + "_image";
            out_msg->time_stamp = bag_message->time_stamp;
            out_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            out_msg->serialized_data->buffer = image_serialized.get_rcl_serialized_message().buffer;
            out_msg->serialized_data->buffer_length =
              image_serialized.get_rcl_serialized_message().buffer_length;
            out_msg->serialized_data->buffer_capacity =
              image_serialized.get_rcl_serialized_message().buffer_capacity;
            out_msg->serialized_data->allocator =
              image_serialized.get_rcl_serialized_message().allocator;

            writer.write(out_msg);
          } else {
            // Initialize encoder for this topic if needed
            auto & encoder = encoders[bag_message->topic_name];
            if (!encoder) {
              encoder = std::make_unique<ffmpeg_encoder_decoder::Encoder>();

              // Configure for lossless grayscale encoding
              encoder->setCVBridgeTargetFormat("mono8");
              encoder->setAVSourcePixelFormat("gray");
              encoder->setEncoder(encoder_name);
              encoder->addAVOption("preset", "ultrafast");
              encoder->addAVOption("crf", "0");  // Lossless

              // Callback when encoded packet is ready
              auto callback = [&writer, &ffmpeg_serializer,
                               topic_name = bag_message->topic_name](
                                const std::string & /*frame_id*/, const rclcpp::Time & stamp,
                                const std::string & codec, uint32_t width, uint32_t height,
                                uint64_t pts, uint8_t flags, uint8_t * data, size_t sz) {
                ffmpeg_image_transport_msgs::msg::FFMPEGPacket pkt_msg;
                pkt_msg.header.stamp = stamp;
                pkt_msg.encoding = codec;
                pkt_msg.width = width;
                pkt_msg.height = height;
                pkt_msg.pts = pts;
                pkt_msg.flags = flags;
                pkt_msg.data.assign(data, data + sz);

                rclcpp::SerializedMessage pkt_serialized;
                ffmpeg_serializer.serialize_message(&pkt_msg, &pkt_serialized);

                auto out_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                out_msg->topic_name = topic_name + "_ffmpeg";
                out_msg->time_stamp =
                  stamp.nanoseconds();
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

              if (!encoder->initialize(image.width, image.height, callback, image.encoding)) {
                throw std::runtime_error("Failed to initialize encoder");
              }
            }

            // Encode the image
            encoder->encodeImage(image);
          }

          converted_count++;

          if (converted_count % 100 == 0) {
            std::cout << "Converted " << converted_count << " messages...\r" << std::flush;
          }
        } catch (const std::exception & e) {
          std::cerr << "Warning: Failed to convert message on " << bag_message->topic_name << ": "
                    << e.what() << "\n";
        }
      } else {
        // Copy message as-is
        writer.write(bag_message);
        copied_count++;
      }
    }

    // Flush all encoders
    for (auto & [topic, encoder] : encoders) {
      if (encoder) {
        encoder->flush();
      }
    }

    std::cout << "\nDone!\n";
    std::cout << "Converted: " << converted_count << " messages\n";
    std::cout << "Copied:    " << copied_count << " messages\n";

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
