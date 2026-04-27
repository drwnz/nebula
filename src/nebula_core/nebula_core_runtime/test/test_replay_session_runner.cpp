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

#include <nebula_core_runtime/replay_session_runner.hpp>

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>

namespace nebula::drivers::test
{
namespace fs = boost::filesystem;

class TestReplaySessionRunner : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = fs::temp_directory_path() / fs::unique_path();
    fs::create_directories(test_dir_);
    
    char* workspace_path = std::getenv("COLCON_PREFIX_PATH");
    if (workspace_path) {
        std::string paths = workspace_path;
        size_t start = 0;
        size_t end = paths.find(':');
        while (end != std::string::npos) {
            fs::path p = fs::path(paths.substr(start, end - start)) / "nebula_sample_decoders" / "lib" / "libnebula_sample_decoders_plugin.so";
            if (fs::exists(p)) {
                plugin_library_path_ = p.string();
                break;
            }
            start = end + 1;
            end = paths.find(':', start);
        }
        if (plugin_library_path_.empty()) {
             fs::path p = fs::path(paths.substr(start)) / "nebula_sample_decoders" / "lib" / "libnebula_sample_decoders_plugin.so";
             if (fs::exists(p)) {
                 plugin_library_path_ = p.string();
             }
        }
    }
  }

  void TearDown() override
  {
    fs::remove_all(test_dir_);
  }

  fs::path test_dir_;
  std::string plugin_library_path_;
};

TEST_F(TestReplaySessionRunner, ReplaySampleSensor)
{
  if (plugin_library_path_.empty() || !fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found";
  }

  // Create a dummy descriptor
  fs::path descriptor_path = test_dir_ / "nebula_sample_decoders.json";
  std::ofstream ofs(descriptor_path.string());
  ofs << R"({
    "vendor": "nebula",
    "package": "nebula_sample_decoders",
    "library": ")" << plugin_library_path_ << R"(",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Sample"]
  })";
  ofs.close();

  auto registry = std::make_shared<SensorRegistry>();
  registry->load_registry({test_dir_.string()});

  ReplaySessionRunner runner(registry);
  
  ReplaySessionConfig config;
  config.model = SensorModel::SAMPLE;
  config.pcap_file = "non_existent.pcap"; // PcapPacketSource will fail to open but runner should still configure
  config.sensor_config.sensor_model = SensorModel::SAMPLE;
  config.sensor_config.data_port = 2368;

  EXPECT_NO_THROW(runner.configure(config));
}

}  // namespace nebula::drivers::test
