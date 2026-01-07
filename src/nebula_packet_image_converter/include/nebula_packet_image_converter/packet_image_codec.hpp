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

#ifndef NEBULA_PACKET_IMAGE_CONVERTER__PACKET_IMAGE_CODEC_HPP_
#define NEBULA_PACKET_IMAGE_CONVERTER__PACKET_IMAGE_CODEC_HPP_

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace nebula::packet_image_converter
{

/**
 * @brief Standalone codec for converting between packet messages and sensor_msgs::Image.
 *        The image format stores each packet as a row:
 *        - 4 bytes: stamp.sec (little-endian)
 *        - 4 bytes: stamp.nanosec (little-endian)
 *        - 4 bytes: size (for PandarPacket, actual data size)
 *        - N bytes: packet data
 */
class PacketImageCodec
{
public:
  // Row width for PandarPacket: 8 bytes timestamp + 4 bytes size + 1500 bytes data
  static constexpr size_t PANDAR_ROW_WIDTH = 8 + 4 + 1500;

  /**
   * @brief Convert PandarScan message to a mono8 Image.
   */
  static sensor_msgs::msg::Image pandar_scan_to_image(const pandar_msgs::msg::PandarScan & scan)
  {
    if (scan.packets.empty()) {
      throw std::runtime_error("Cannot convert empty PandarScan to image");
    }

    const size_t num_packets = scan.packets.size();
    const size_t image_size = num_packets * PANDAR_ROW_WIDTH;

    sensor_msgs::msg::Image image;
    image.header = scan.header;
    image.height = static_cast<uint32_t>(num_packets);
    image.width = static_cast<uint32_t>(PANDAR_ROW_WIDTH);
    image.encoding = "mono8";
    image.is_bigendian = 0;
    image.step = static_cast<uint32_t>(PANDAR_ROW_WIDTH);
    image.data.resize(image_size);

    uint8_t * img_ptr = image.data.data();

    for (const auto & packet : scan.packets) {
      // Store timestamp (8 bytes: sec + nanosec)
      uint32_t sec = static_cast<uint32_t>(packet.stamp.sec);
      uint32_t nanosec = packet.stamp.nanosec;

      std::memcpy(img_ptr, &sec, sizeof(sec));
      img_ptr += sizeof(sec);
      std::memcpy(img_ptr, &nanosec, sizeof(nanosec));
      img_ptr += sizeof(nanosec);

      // Store size (4 bytes)
      uint32_t size = packet.size;
      std::memcpy(img_ptr, &size, sizeof(size));
      img_ptr += sizeof(size);

      // Store data (fixed 1500 bytes)
      std::memcpy(img_ptr, packet.data.data(), 1500);
      img_ptr += 1500;
    }

    return image;
  }

  /**
   * @brief Convert a mono8 Image back to PandarScan.
   */
  static pandar_msgs::msg::PandarScan image_to_pandar_scan(const sensor_msgs::msg::Image & image)
  {
    if (image.encoding != "mono8") {
      throw std::runtime_error("Image must be mono8 encoding");
    }
    if (image.width != PANDAR_ROW_WIDTH) {
      throw std::runtime_error("Image width must match PandarPacket row width");
    }

    const size_t num_packets = image.height;

    pandar_msgs::msg::PandarScan scan;
    scan.header = image.header;
    scan.packets.resize(num_packets);

    const uint8_t * img_ptr = image.data.data();

    for (size_t i = 0; i < num_packets; ++i) {
      auto & packet = scan.packets[i];

      // Read timestamp
      uint32_t sec, nanosec;
      std::memcpy(&sec, img_ptr, sizeof(sec));
      img_ptr += sizeof(sec);
      std::memcpy(&nanosec, img_ptr, sizeof(nanosec));
      img_ptr += sizeof(nanosec);

      packet.stamp.sec = static_cast<int32_t>(sec);
      packet.stamp.nanosec = nanosec;

      // Read size
      uint32_t size;
      std::memcpy(&size, img_ptr, sizeof(size));
      img_ptr += sizeof(size);
      packet.size = size;

      // Read data (fixed 1500 bytes)
      std::memcpy(packet.data.data(), img_ptr, 1500);
      img_ptr += 1500;
    }

    return scan;
  }

  /**
   * @brief Convert NebulaPackets message to a mono8 Image.
   */
  static sensor_msgs::msg::Image packets_to_image(const nebula_msgs::msg::NebulaPackets & packets)
  {
    if (packets.packets.empty()) {
      throw std::runtime_error("Cannot convert empty packets to image");
    }

    const size_t packet_data_size = packets.packets[0].data.size();
    const size_t row_width = 8 + packet_data_size;  // 8 bytes timestamp + data
    const size_t num_packets = packets.packets.size();
    const size_t image_size = num_packets * row_width;

    sensor_msgs::msg::Image image;
    image.header = packets.header;
    image.height = static_cast<uint32_t>(num_packets);
    image.width = static_cast<uint32_t>(row_width);
    image.encoding = "mono8";
    image.is_bigendian = 0;
    image.step = static_cast<uint32_t>(row_width);
    image.data.resize(image_size);

    uint8_t * img_ptr = image.data.data();

    for (const auto & packet : packets.packets) {
      // Store timestamp (8 bytes: sec + nanosec)
      uint32_t sec = static_cast<uint32_t>(packet.stamp.sec);
      uint32_t nanosec = packet.stamp.nanosec;

      std::memcpy(img_ptr, &sec, sizeof(sec));
      img_ptr += sizeof(sec);
      std::memcpy(img_ptr, &nanosec, sizeof(nanosec));
      img_ptr += sizeof(nanosec);

      // Store data (pad with zeros if size mismatch)
      if (packet.data.size() == packet_data_size) {
        std::memcpy(img_ptr, packet.data.data(), packet_data_size);
      } else {
        std::memset(img_ptr, 0, packet_data_size);
      }
      img_ptr += packet_data_size;
    }

    return image;
  }

  /**
   * @brief Convert a mono8 Image back to NebulaPackets.
   */
  static nebula_msgs::msg::NebulaPackets image_to_packets(const sensor_msgs::msg::Image & image)
  {
    if (image.encoding != "mono8") {
      throw std::runtime_error("Image must be mono8 encoding");
    }
    if (image.width < 8) {
      throw std::runtime_error("Image width must be at least 8 bytes (timestamp)");
    }

    const size_t packet_data_size = image.width - 8;
    const size_t num_packets = image.height;

    nebula_msgs::msg::NebulaPackets packets;
    packets.header = image.header;
    packets.packets.resize(num_packets);

    const uint8_t * img_ptr = image.data.data();

    for (size_t i = 0; i < num_packets; ++i) {
      auto & packet = packets.packets[i];

      // Read timestamp
      uint32_t sec, nanosec;
      std::memcpy(&sec, img_ptr, sizeof(sec));
      img_ptr += sizeof(sec);
      std::memcpy(&nanosec, img_ptr, sizeof(nanosec));
      img_ptr += sizeof(nanosec);

      packet.stamp.sec = static_cast<int32_t>(sec);
      packet.stamp.nanosec = nanosec;

      // Read data
      packet.data.resize(packet_data_size);
      std::memcpy(packet.data.data(), img_ptr, packet_data_size);
      img_ptr += packet_data_size;
    }

    return packets;
  }
};

}  // namespace nebula::packet_image_converter

#endif  // NEBULA_PACKET_IMAGE_CONVERTER__PACKET_IMAGE_CODEC_HPP_
