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

#include "sample_plugin_test_utils.hpp"

#include <nebula_core_runtime/live_transport_graph.hpp>
#include <nebula_core_runtime/sensor_registry.hpp>

#include <boost/filesystem.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace nebula::drivers::test
{
namespace fs = boost::filesystem;

class TestLiveTransportGraphWithSamplePlugin : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = fs::temp_directory_path() / fs::unique_path();
    fs::create_directories(test_dir_);

    plugin_library_path_ = find_sample_plugin_library();
    ASSERT_NO_THROW(dependency_handle_ = load_sample_plugin_dependency(plugin_library_path_));
  }

  void TearDown() override
  {
    fs::remove_all(test_dir_);
    if (dependency_handle_) {
      dlclose(dependency_handle_);
      dependency_handle_ = nullptr;
    }
  }

  void write_sample_descriptor() const
  {
    std::ofstream ofs((test_dir_ / "nebula_sample_decoders.json").string());
    ofs << R"({
    "vendor": "nebula",
    "package": "nebula_sample_decoders",
    "library": ")"
        << plugin_library_path_ << R"(",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Sample"]
  })";
  }

  fs::path test_dir_;
  std::string plugin_library_path_;
  void * dependency_handle_{nullptr};
};

TEST(TestLiveTransportGraph, HttpGetThrowsForUnknownEndpoint)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_THROW(graph.http_get("nonexistent"), std::invalid_argument);
}

TEST(TestLiveTransportGraph, HttpPostThrowsForUnknownEndpoint)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_THROW(graph.http_post("nonexistent", "body"), std::invalid_argument);
}

TEST(TestLiveTransportGraph, StartStopOnUnconfiguredGraphIsNoop)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_NO_THROW(graph.start());
  EXPECT_NO_THROW(graph.stop());
}

TEST(TestLiveTransportGraph, SetCallbacksDoNotCrash)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_NO_THROW(graph.set_output_callback([](const SensorDecodedOutput &) {}));
  EXPECT_NO_THROW(graph.set_error_callback([](const SensorError &) {}));
  EXPECT_NO_THROW(graph.set_progress_callback([](const SensorProgress &) {}));
}

TEST(TestLiveTransportGraph, ConfigureThrowsForUnknownModel)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  LiveSessionConfig config;
  config.model = SensorModel::VELODYNE_VLP16;
  EXPECT_THROW(graph.configure(config), std::runtime_error);
}

TEST_F(TestLiveTransportGraphWithSamplePlugin, ConfigureSampleUdpGraph)
{
  if (plugin_library_path_.empty() || !fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found";
  }

  write_sample_descriptor();

  auto registry = std::make_shared<SensorRegistry>();
  registry->load_registry({test_dir_.string()});

  LiveTransportGraph graph(registry);
  LiveSessionConfig config;
  config.model = SensorModel::SAMPLE;
  config.sensor_config.sensor_model = SensorModel::SAMPLE;
  config.sensor_config.host_ip = "127.0.0.1";
  config.sensor_config.sensor_ip = "127.0.0.1";
  config.sensor_config.data_port = 6263;

  EXPECT_NO_THROW(graph.configure(config));
  EXPECT_NO_THROW(graph.stop());
}

TEST(TestLiveTransportGraph, ConfigureAllSupportedTransportKinds)
{
  const fs::path test_dir = fs::temp_directory_path() / fs::unique_path();
  fs::create_directories(test_dir);

  const std::string plugin_library_path = NEBULA_RUNTIME_TEST_LIVE_TRANSPORT_PLUGIN;
  ASSERT_FALSE(plugin_library_path.empty());
  ASSERT_TRUE(fs::exists(plugin_library_path));

  std::ofstream ofs((test_dir / "nebula_live_transport_test_plugin.json").string());
  ofs << R"({
    "vendor": "nebula",
    "package": "nebula_live_transport_test_plugin",
    "library": ")"
      << plugin_library_path << R"(",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Sample"]
  })";
  ofs.close();

  auto registry = std::make_shared<SensorRegistry>();
  registry->load_registry({test_dir.string()});

  LiveTransportGraph graph(registry);
  LiveSessionConfig config;
  config.model = SensorModel::SAMPLE;
  config.sensor_config.sensor_model = SensorModel::SAMPLE;
  config.sensor_config.host_ip = "127.0.0.1";
  config.sensor_config.sensor_ip = "127.0.0.1";
  config.sensor_config.data_port = 6263;
  config.sensor_config.extra_params["can_interface"] = "vcan0";

  EXPECT_NO_THROW(graph.configure(config));
  EXPECT_THROW(graph.http_get("missing"), std::invalid_argument);
  EXPECT_EQ(graph.get_router_metrics().processed_packets, 0u);
  EXPECT_NO_THROW(graph.stop());

  fs::remove_all(test_dir);
}

}  // namespace nebula::drivers::test
