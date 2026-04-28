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

#include <nebula_core_runtime/sensor_registry.hpp>

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>

namespace nebula::drivers::test
{
namespace fs = boost::filesystem;

class TestSamplePluginIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // We expect the library to be in nebula/install/nebula_sample_decoders/lib/libnebula_sample_decoders_plugin.so
    // But during build/test it might be in different places. 
    // For simplicity, we assume we can find it relative to the current working directory if run from nebula/
    
    std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
    for (const auto & env_name : prefix_envs) {
        char* env_val = std::getenv(env_name.c_str());
        if (env_val) {
            std::vector<std::string> prefixes;
            boost::split(prefixes, env_val, boost::is_any_of(":"));
            for (const auto & prefix : prefixes) {
                if (prefix.empty()) continue;
                // Try common lib
                fs::path p1 = fs::path(prefix) / "lib" / "libnebula_sample_decoders_plugin.so";
                if (fs::exists(p1)) {
                    plugin_library_path_ = p1.string();
                    break;
                }
                // Try isolated lib
                fs::path p2 = fs::path(prefix) / "nebula_sample_decoders" / "lib" / "libnebula_sample_decoders_plugin.so";
                if (fs::exists(p2)) {
                    plugin_library_path_ = p2.string();
                    break;
                }
            }
            if (!plugin_library_path_.empty()) break;
        }
    }

    if (plugin_library_path_.empty()) {
        // Fallback to a common location
        plugin_library_path_ = "install/nebula_sample_decoders/lib/libnebula_sample_decoders_plugin.so";
    }
  }

  std::string plugin_library_path_;
};

TEST_F(TestSamplePluginIntegration, LoadAndRunSamplePlugin)
{
  if (!fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found at " << plugin_library_path_;
  }

  SensorRegistry registry;
  
  SensorPluginMetadata metadata;
  metadata.vendor = "nebula";
  metadata.package_name = "nebula_sample_decoders";
  metadata.library_path = plugin_library_path_;
  metadata.supported_models = {SensorModel::SAMPLE};

  auto plugin = registry.load_plugin(metadata);
  ASSERT_NE(plugin, nullptr);
  EXPECT_EQ(plugin->metadata().vendor, "nebula");

  auto runtime = plugin->create_decoder_runtime();
  ASSERT_NE(runtime, nullptr);

  SensorConfiguration config;
  config.sensor_model = SensorModel::SAMPLE;
  config.data_port = 2368;
  runtime->configure(config);

  bool output_received = false;
  runtime->set_output_callback([&](const SensorDecodedOutput & output) {
    EXPECT_EQ(output.kind, SensorOutputKind::PointCloud);
    output_received = true;
  });

  // SampleDecoder needs 10 packets to emit a pointcloud
  for (int i = 0; i < 10; ++i) {
    SensorPacket packet;
    packet.transport = SensorTransportKind::UDP;
    packet.channel = SensorPacketChannel::Data;
    packet.payload = {0xde, 0xad, 0xbe, 0xef};
    auto result = runtime->process_packet(packet);
    EXPECT_EQ(result, SensorPacketResult::Success);
  }

  EXPECT_TRUE(output_received);
}

}  // namespace nebula::drivers::test
