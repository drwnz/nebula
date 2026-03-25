// Copyright 2024 TIER IV, Inc.

#include "nebula_robosense_decoders/robosense_info_driver.hpp"

#include "nebula_robosense_decoders/decoders/bpearl_v3.hpp"
#include "nebula_robosense_decoders/decoders/bpearl_v4.hpp"
#include "nebula_robosense_decoders/decoders/e1.hpp"
#include "nebula_robosense_decoders/decoders/em4.hpp"
#include "nebula_robosense_decoders/decoders/emx.hpp"
#include "nebula_robosense_decoders/decoders/helios.hpp"
#include "nebula_robosense_decoders/decoders/robosense_info_decoder.hpp"

#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{
// RobosenseInfoDecoder specialization for EM4 to handle DIFOP1 and DIFOP2
template <>
class RobosenseInfoDecoder<EM4> : public RobosenseInfoDecoderBase
{
protected:
  EM4 sensor_{};
  robosense_packet::em4::CombinedInfo packet_{};
  rclcpp::Logger logger_;

public:
  RobosenseInfoDecoder() : logger_(rclcpp::get_logger("RobosenseInfoDecoderEM4"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  }

  bool parse_packet(const std::vector<uint8_t> & raw_packet) override
  {
    if (raw_packet.size() == 1248) {  // DIFOP1
      std::memcpy(&packet_.packet1, raw_packet.data(), sizeof(robosense_packet::em4::InfoPacket));
      packet_.packet1_received = true;
      return true;
    } else if (raw_packet.size() == 1162) {  // DIFOP2
      std::memcpy(&packet_.packet2, raw_packet.data(), sizeof(robosense_packet::em4::InfoPacket2));
      packet_.packet2_received = true;
      return true;
    }
    return false;
  }

  std::map<std::string, std::string> get_sensor_info() override
  {
    return sensor_.get_sensor_info(packet_);
  }
  ReturnMode get_return_mode() override { return sensor_.get_return_mode(packet_); }
  RobosenseCalibrationConfiguration get_sensor_calibration() override
  {
    return sensor_.get_sensor_calibration(packet_);
  }
  bool get_sync_status() override { return sensor_.get_sync_status(packet_); }
};

// RobosenseInfoDecoder specialization for EMX to handle 256-byte DIFOP
template <>
class RobosenseInfoDecoder<EMX> : public RobosenseInfoDecoderBase
{
protected:
  EMX sensor_{};
  robosense_packet::emx::CombinedInfo packet_{};
  rclcpp::Logger logger_;

public:
  RobosenseInfoDecoder() : logger_(rclcpp::get_logger("RobosenseInfoDecoderEMX"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  }

  bool parse_packet(const std::vector<uint8_t> & raw_packet) override
  {
    if (raw_packet.size() == 654) {  // DIFOP1 (New)
      std::memcpy(&packet_.packet1, raw_packet.data(), sizeof(robosense_packet::emx::InfoPacket));
      packet_.packet1_received = true;
      return true;
    } else if (raw_packet.size() == 500) {  // DIFOP2 (New)
      std::memcpy(&packet_.packet2, raw_packet.data(), sizeof(robosense_packet::emx::InfoPacket2));
      packet_.packet2_received = true;
      return true;
    } else if (raw_packet.size() == 256) {  // Standard DIFOP
      std::memcpy(
        &packet_.packet256, raw_packet.data(), sizeof(robosense_packet::emx::InfoPacket256));
      packet_.packet256_received = true;
      return true;
    }
    return false;
  }

  std::map<std::string, std::string> get_sensor_info() override
  {
    return sensor_.get_sensor_info(packet_);
  }
  ReturnMode get_return_mode() override { return sensor_.get_return_mode(packet_); }
  RobosenseCalibrationConfiguration get_sensor_calibration() override
  {
    return sensor_.get_sensor_calibration(packet_);
  }
  bool get_sync_status() override { return sensor_.get_sync_status(packet_); }
};

RobosenseInfoDriver::RobosenseInfoDriver(
  const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration)
{
  // Initialize proper parser from cloud config's model and echo mode
  driver_status_ = ::nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = ::nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::ROBOSENSE_BPEARL_V3:
      info_decoder_.reset(new RobosenseInfoDecoder<BpearlV3>());
      break;
    case SensorModel::ROBOSENSE_BPEARL_V4:
      info_decoder_.reset(new RobosenseInfoDecoder<BpearlV4>());
      break;
    case SensorModel::ROBOSENSE_HELIOS:
      info_decoder_.reset(new RobosenseInfoDecoder<Helios>());
      break;
    case SensorModel::ROBOSENSE_E1:
      info_decoder_.reset(new RobosenseInfoDecoder<E1>());
      break;
    case SensorModel::ROBOSENSE_EM4:
      info_decoder_.reset(new RobosenseInfoDecoder<EM4>());
      break;
    case SensorModel::ROBOSENSE_EMX:
      info_decoder_.reset(new RobosenseInfoDecoder<EMX>());
      break;

    default:
      driver_status_ = ::nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
      break;
  }
}

Status RobosenseInfoDriver::get_status()
{
  return driver_status_;
}

Status RobosenseInfoDriver::decode_info_packet(const std::vector<uint8_t> & packet)
{
  const auto parsed = info_decoder_->parse_packet(packet);
  if (parsed) return ::nebula::Status::OK;
  return ::nebula::Status::ERROR_1;
}

std::map<std::string, std::string> RobosenseInfoDriver::get_sensor_info()
{
  return info_decoder_->get_sensor_info();
}

ReturnMode RobosenseInfoDriver::get_return_mode()
{
  return info_decoder_->get_return_mode();
}

RobosenseCalibrationConfiguration RobosenseInfoDriver::get_sensor_calibration()
{
  return info_decoder_->get_sensor_calibration();
}

bool RobosenseInfoDriver::get_sync_status()
{
  return info_decoder_->get_sync_status();
}

}  // namespace nebula::drivers
