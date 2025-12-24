// Copyright 2025 TIER IV, Inc.

#ifndef NEBULA_PACKET_IMAGE_CONVERTER__IMAGE_TO_NEBULA_PACKETS_HPP_
#define NEBULA_PACKET_IMAGE_CONVERTER__IMAGE_TO_NEBULA_PACKETS_HPP_

#include <nebula_core_ros/agnocast_wrapper/nebula_agnocast_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nebula::packet_image_converter
{

class ImageToNebulaPackets : public rclcpp::Node
{
public:
  explicit ImageToNebulaPackets(const rclcpp::NodeOptions & options);

private:
  void image_callback(NEBULA_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::Image) && msg);

  NEBULA_SUBSCRIPTION_PTR(sensor_msgs::msg::Image) sub_image_;
  NEBULA_PUBLISHER_PTR(nebula_msgs::msg::NebulaPackets) pub_packets_;
};

}  // namespace nebula::packet_image_converter

#endif  // NEBULA_PACKET_IMAGE_CONVERTER__IMAGE_TO_NEBULA_PACKETS_HPP_
