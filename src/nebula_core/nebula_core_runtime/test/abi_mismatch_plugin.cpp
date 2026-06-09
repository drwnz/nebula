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
#include <vector>

namespace nebula::drivers::test
{
class AbiMismatchPlugin : public SensorPlugin
{
public:
  SensorPluginMetadata metadata() const override
  {
    SensorPluginMetadata metadata;
    metadata.package_name = "abi_mismatch";
    return metadata;
  }

  std::vector<PacketChannelRequirement> packet_requirements(
    const SensorConfiguration & /*config*/) const override
  {
    return {};
  }

  std::vector<LiveTransportRequirement> live_transport_requirements(
    const SensorConfiguration & /*config*/) const override
  {
    return {};
  }

  std::unique_ptr<SensorDecoderRuntime> create_decoder_runtime() const override { return nullptr; }
};
}  // namespace nebula::drivers::test

extern "C" nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::test::AbiMismatchPlugin();
}

extern "C" void destroy_nebula_sensor_plugin(nebula::drivers::SensorPlugin * plugin)
{
  delete plugin;
}

extern "C" uint32_t nebula_plugin_abi_version()
{
  return nebula::drivers::kNebulaPluginAbiVersion + 1;
}
