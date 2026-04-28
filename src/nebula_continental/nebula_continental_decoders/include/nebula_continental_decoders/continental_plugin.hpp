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

#ifndef NEBULA_CONTINENTAL_PLUGIN_HPP
#define NEBULA_CONTINENTAL_PLUGIN_HPP

#include <nebula_core_common/radar_types.hpp>
#include <nebula_core_decoders/sensor_plugin.hpp>

#include <memory>

namespace nebula::drivers
{

class ContinentalARS548SensorDecoderRuntime : public SensorDecoderRuntime
{
public:
  ContinentalARS548SensorDecoderRuntime();
  ~ContinentalARS548SensorDecoderRuntime() override;
  void configure(const SensorConfiguration & config) override;
  void set_output_callback(SensorOutputCallback callback) override { output_callback_ = callback; }
  void set_error_callback(SensorErrorCallback callback) override { error_callback_ = callback; }
  void set_progress_callback(SensorProgressCallback callback) override { progress_callback_ = callback; }
  SensorPacketResult process_packet(const SensorPacket & packet) override;
  void flush() override {}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  SensorOutputCallback output_callback_;
  SensorErrorCallback error_callback_;
  SensorProgressCallback progress_callback_;
  SensorProgress progress_;
};

class ContinentalSRR520SensorDecoderRuntime : public SensorDecoderRuntime
{
public:
  ContinentalSRR520SensorDecoderRuntime();
  ~ContinentalSRR520SensorDecoderRuntime() override;
  void configure(const SensorConfiguration & config) override;
  void set_output_callback(SensorOutputCallback callback) override { output_callback_ = callback; }
  void set_error_callback(SensorErrorCallback callback) override { error_callback_ = callback; }
  void set_progress_callback(SensorProgressCallback callback) override { progress_callback_ = callback; }
  SensorPacketResult process_packet(const SensorPacket & packet) override;
  void flush() override {}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  SensorOutputCallback output_callback_;
  SensorErrorCallback error_callback_;
  SensorProgressCallback progress_callback_;
  SensorProgress progress_;
};

class ContinentalSensorPlugin : public SensorPlugin
{
public:
  SensorPluginMetadata metadata() const override;
  std::vector<SensorModelInfo> supported_models() const override;
  std::vector<PacketChannelRequirement> packet_requirements(const SensorConfiguration & config) const override;
  std::vector<LiveTransportRequirement> live_transport_requirements(const SensorConfiguration & config) const override;
  std::unique_ptr<SensorDecoderRuntime> create_decoder_runtime() const override;
  
  mutable SensorModel current_model_{SensorModel::UNKNOWN};
};

}  // namespace nebula::drivers

#endif  // NEBULA_CONTINENTAL_PLUGIN_HPP
