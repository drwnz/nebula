// Copyright 2025 TIER IV, Inc.

#include "nebula_packet_image_converter/nebula_packets_to_image.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

namespace nebula::packet_image_converter
{

NebulaPacketsToImage::NebulaPacketsToImage(const rclcpp::NodeOptions & options)
: Node("nebula_packets_to_image", options)
{
  pub_image_ = NEBULA_CREATE_PUBLISHER2(sensor_msgs::msg::Image, this, "nebula_packets_image", 10);
  sub_packets_ = NEBULA_CREATE_SUBSCRIPTION(
    nebula_msgs::msg::NebulaPackets, this, "nebula_packets", 10,
    std::bind(&NebulaPacketsToImage::packets_callback, this, std::placeholders::_1),
    NEBULA_SUBSCRIPTION_OPTIONS());
}

void NebulaPacketsToImage::packets_callback(
  NEBULA_MESSAGE_UNIQUE_PTR(nebula_msgs::msg::NebulaPackets) && msg)
{
  if (msg->packets.empty()) {
    return;
  }

  // Determine image dimensions
  // Each row = 8 bytes timestamp + N bytes data
  size_t packet_data_size = msg->packets[0].data.size();
  size_t row_width = 8 + packet_data_size;
  size_t num_packets = msg->packets.size();
  size_t image_size = num_packets * row_width;

  auto image_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_image_);
  if (!image_msg) {
    RCLCPP_ERROR(get_logger(), "Failed to allocate output message");
    return;
  }

  image_msg->header = msg->header;
  image_msg->height = num_packets;
  image_msg->width = row_width;
  image_msg->encoding = "mono8";
  image_msg->is_bigendian = 0;
  image_msg->step = row_width;
  // Allocation of data buffer
  // If we had efficient loaned messages with flexible size, we'd use that.
  // Standard ROS 2 vector resize is normal.
  image_msg->data.resize(image_size);

  uint8_t * img_ptr = image_msg->data.data();

  for (const auto & packet : msg->packets) {
    // 1. Copy Timestamp (8 bytes)
    // We store sec (4 bytes) and nanosec (4 bytes)
    // Little-endian storage
    uint32_t sec = static_cast<uint32_t>(packet.stamp.sec);
    uint32_t nanosec = packet.stamp.nanosec;

    std::memcpy(img_ptr, &sec, sizeof(sec));
    img_ptr += sizeof(sec);
    std::memcpy(img_ptr, &nanosec, sizeof(nanosec));
    img_ptr += sizeof(nanosec);

    // 2. Copy Data
    if (packet.data.size() != packet_data_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Packet data size mismatch within batch! Dropping packet data for index.");
      std::memset(img_ptr, 0, packet_data_size);
    } else {
      std::memcpy(img_ptr, packet.data.data(), packet_data_size);
    }
    img_ptr += packet_data_size;
  }

  pub_image_->publish(std::move(image_msg));
}

}  // namespace nebula::packet_image_converter

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::packet_image_converter::NebulaPacketsToImage)
