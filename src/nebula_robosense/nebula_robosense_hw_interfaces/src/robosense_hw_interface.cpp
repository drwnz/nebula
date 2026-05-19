// Copyright 2024 TIER IV, Inc.

#include "nebula_robosense_hw_interfaces/robosense_hw_interface.hpp"

#include "nebula_core_common/util/string_conversions.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace nebula::drivers
{
RobosenseHwInterface::RobosenseHwInterface(const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger)
{
}

void RobosenseHwInterface::receive_sensor_packet_callback(std::vector<uint8_t> & buffer)
{
  if (!scan_reception_callback_) {
    return;
  }

  scan_reception_callback_(buffer);
}

void RobosenseHwInterface::receive_info_packet_callback(std::vector<uint8_t> & buffer)
{
  if (!info_reception_callback_) {
    return;
  }

  info_reception_callback_(buffer);
}

Status RobosenseHwInterface::sensor_interface_start()
{
  try {
    logger_->info(
      "Starting UDP server for data packets on: " + sensor_configuration_->sensor_ip + ": " +
      std::to_string(sensor_configuration_->data_port));

    cloud_udp_socket_ = std::make_unique<connections::UdpSocket>(
      connections::UdpSocket::Builder(
        sensor_configuration_->host_ip, sensor_configuration_->data_port)
        .limit_to_sender(sensor_configuration_->sensor_ip, sensor_configuration_->data_port)
        .bind());

    cloud_udp_socket_->subscribe(
      [this](std::vector<uint8_t> & buffer, const connections::UdpSocket::RxMetadata &) {
        this->receive_sensor_packet_callback(buffer);
      });
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    logger_->error(
      util::to_string(status) + " " + sensor_configuration_->sensor_ip + "," +
      std::to_string(sensor_configuration_->data_port) + ": " + ex.what());
    return status;
  }
  return Status::OK;
}

Status RobosenseHwInterface::start_info_socket(
  std::unique_ptr<connections::UdpSocket> & socket, uint16_t port, const std::string & label)
{
  try {
    logger_->info(
      "Starting UDP server for " + label + " packets on: " + sensor_configuration_->sensor_ip +
      ": " + std::to_string(port));

    socket = std::make_unique<connections::UdpSocket>(
      connections::UdpSocket::Builder(sensor_configuration_->host_ip, port)
        .limit_to_sender(sensor_configuration_->sensor_ip, port)
        .bind());

    socket->subscribe(
      [this](std::vector<uint8_t> & buffer, const connections::UdpSocket::RxMetadata &) {
        this->receive_info_packet_callback(buffer);
      });
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    logger_->error(
      util::to_string(status) + " " + sensor_configuration_->sensor_ip + "," +
      std::to_string(port) + ": " + ex.what());
    return status;
  }

  return Status::OK;
}

Status RobosenseHwInterface::info_interface_start()
{
  const auto primary_status =
    start_info_socket(info_udp_socket_, sensor_configuration_->gnss_port, "info/DIFOP");
  if (primary_status != Status::OK) {
    return primary_status;
  }

  if (
    sensor_configuration_->difop2_port != 0 &&
    sensor_configuration_->difop2_port != sensor_configuration_->gnss_port) {
    return start_info_socket(difop2_udp_socket_, sensor_configuration_->difop2_port, "DIFOP2");
  }

  return Status::OK;
}

Status RobosenseHwInterface::set_sensor_configuration(
  std::shared_ptr<const RobosenseSensorConfiguration> sensor_configuration)
{
  if (!(sensor_configuration->sensor_model == SensorModel::ROBOSENSE_BPEARL_V3 ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_BPEARL_V4 ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_E1 ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_EM4 ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_EMX)) {
    return Status::INVALID_SENSOR_MODEL;
  }

  sensor_configuration_ = sensor_configuration;

  return Status::OK;
}

Status RobosenseHwInterface::register_scan_callback(
  std::function<void(std::vector<uint8_t> &)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

Status RobosenseHwInterface::register_info_callback(
  std::function<void(std::vector<uint8_t> &)> info_callback)
{
  info_reception_callback_ = std::move(info_callback);
  return Status::OK;
}
}  // namespace nebula::drivers
