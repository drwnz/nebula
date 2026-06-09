// Copyright 2026 TIER IV, Inc.
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

#include <nebula_core_decoders/sensor_plugin.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers::test
{
namespace
{
class LiveTransportTestRuntime final : public SensorDecoderRuntime
{
public:
  void configure(const SensorConfiguration &) override {}
  void set_output_callback(SensorOutputCallback callback) override { output_callback_ = callback; }
  void set_error_callback(SensorErrorCallback callback) override { error_callback_ = callback; }
  void set_progress_callback(SensorProgressCallback callback) override
  {
    progress_callback_ = callback;
  }

  SensorPacketResult process_packet(const SensorPacketView &) override
  {
    if (output_callback_) {
      SensorDecodedOutput output;
      output.kind = SensorOutputKind::PointCloud;
      output.sensor_id = "live_transport_test";
      output_callback_(output);
    }
    return SensorPacketResult::Success;
  }

  void flush() override {}

private:
  SensorOutputCallback output_callback_;
  SensorErrorCallback error_callback_;
  SensorProgressCallback progress_callback_;
};

class LiveTransportTestPlugin final : public SensorPlugin
{
public:
  SensorPluginMetadata metadata() const override
  {
    SensorPluginMetadata metadata;
    metadata.vendor = "nebula";
    metadata.package_name = "nebula_live_transport_test_plugin";
    metadata.supported_models = {SensorModel::SAMPLE};
    return metadata;
  }

  std::vector<PacketChannelRequirement> packet_requirements(
    const SensorConfiguration & config) const override
  {
    return {PacketChannelRequirement{
      SensorTransportKind::UDP, SensorPacketChannel::Data, true, config.data_port}};
  }

  std::vector<LiveTransportRequirement> live_transport_requirements(
    const SensorConfiguration & config) const override
  {
    return {
      LiveTransportRequirement{
        SensorTransportKind::UDP, SensorPacketChannel::Data, true, "udp-data", config.data_port},
      LiveTransportRequirement{
        SensorTransportKind::TCP, SensorPacketChannel::Info, false, "tcp-telemetry",
        static_cast<uint16_t>(config.data_port + 1)},
      LiveTransportRequirement{
        SensorTransportKind::CAN, SensorPacketChannel::Status, false, "can-status"},
      LiveTransportRequirement{
        SensorTransportKind::HTTP, SensorPacketChannel::Control, false, "http-control",
        static_cast<uint16_t>(config.data_port + 2), std::string{"/status"}}};
  }

  std::unique_ptr<SensorDecoderRuntime> create_decoder_runtime() const override
  {
    return std::make_unique<LiveTransportTestRuntime>();
  }
};
}  // namespace
}  // namespace nebula::drivers::test

extern "C" uint32_t nebula_plugin_abi_version()
{
  return nebula::drivers::kNebulaPluginAbiVersion;
}

extern "C" nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::test::LiveTransportTestPlugin();
}

extern "C" void destroy_nebula_sensor_plugin(nebula::drivers::SensorPlugin * plugin)
{
  delete plugin;
}
