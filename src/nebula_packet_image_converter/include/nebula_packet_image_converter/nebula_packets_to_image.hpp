// Copyright 2025 TIER IV, Inc.

#ifndef NEBULA_PACKET_IMAGE_CONVERTER__NEBULA_PACKETS_TO_IMAGE_HPP_
#define NEBULA_PACKET_IMAGE_CONVERTER__NEBULA_PACKETS_TO_IMAGE_HPP_

#include <nebula_core_ros/agnocast_wrapper/nebula_agnocast_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nebula::packet_image_converter
{

class NebulaPacketsToImage : public rclcpp::Node
{
public:
  explicit NebulaPacketsToImage(const rclcpp::NodeOptions & options);

private:
  void packets_callback(NEBULA_MESSAGE_UNIQUE_PTR(nebula_msgs::msg::NebulaPackets) && msg);

  NEBULA_SUBSCRIPTION_PTR(nebula_msgs::msg::NebulaPackets) sub_packets_;
  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::Image) pub_image_;
};

}  // namespace nebula::packet_image_converter

#endif  // NEBULA_PACKET_IMAGE_CONVERTER__NEBULA_PACKETS_TO_IMAGE_HPP_
