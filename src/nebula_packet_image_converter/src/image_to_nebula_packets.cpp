// Copyright 2025 TIER IV, Inc.

#include "nebula_packet_image_converter/image_to_nebula_packets.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

namespace nebula::packet_image_converter
{

ImageToNebulaPackets::ImageToNebulaPackets(const rclcpp::NodeOptions & options)
: Node("image_to_nebula_packets", options)
{
  pub_packets_ = NEBULA_CREATE_PUBLISHER2(
    nebula_msgs::msg::NebulaPackets, this, "nebula_packets_reconstructed", 10);
  sub_image_ = NEBULA_CREATE_SUBSCRIPTION(
    sensor_msgs::msg::Image, this, "nebula_packets_image", 10,
    std::bind(&ImageToNebulaPackets::image_callback, this, std::placeholders::_1),
    NEBULA_SUBSCRIPTION_OPTIONS());
}

void ImageToNebulaPackets::image_callback(NEBULA_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::Image) && msg)
{
  if (msg->encoding != "mono8") {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Unsupported image encoding: %s. Expected mono8.",
      msg->encoding.c_str());
    return;
  }

  // row_width = 8 + packet_data_size
  if (msg->step <= 8) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image too narrow to contain packets.");
    return;
  }

  size_t packet_data_size = msg->step - 8;
  size_t num_packets = msg->height;

  auto packets_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_packets_);
  if (!packets_msg) {
    RCLCPP_ERROR(get_logger(), "Failed to allocate output message");
    return;
  }

  packets_msg->header = msg->header;
  packets_msg->packets.reserve(num_packets);

  const uint8_t * img_ptr = msg->data.data();

  for (size_t i = 0; i < num_packets; ++i) {
    nebula_msgs::msg::NebulaPacket packet;

    // 1. Recover Timestamp (8 bytes)
    uint32_t sec = 0;
    uint32_t nanosec = 0;

    std::memcpy(&sec, img_ptr, sizeof(sec));
    img_ptr += sizeof(sec);
    std::memcpy(&nanosec, img_ptr, sizeof(nanosec));
    img_ptr += sizeof(nanosec);

    packet.stamp.sec = static_cast<int32_t>(sec);
    packet.stamp.nanosec = nanosec;

    // 2. Recover Data
    // We would ideally want to avoid this copy, but NebulaPacket has a vector<uint8_t>
    // so we must resize and copy.
    packet.data.resize(packet_data_size);
    std::memcpy(packet.data.data(), img_ptr, packet_data_size);
    img_ptr += packet_data_size;

    packets_msg->packets.emplace_back(std::move(packet));
  }

  pub_packets_->publish(std::move(packets_msg));
}

}  // namespace nebula::packet_image_converter

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::packet_image_converter::ImageToNebulaPackets)
