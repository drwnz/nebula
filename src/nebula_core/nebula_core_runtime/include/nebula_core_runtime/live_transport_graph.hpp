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

#ifndef NEBULA_LIVE_TRANSPORT_GRAPH_HPP
#define NEBULA_LIVE_TRANSPORT_GRAPH_HPP

#include <nebula_core_runtime/sensor_registry.hpp>
#include <nebula_core_runtime/packet_router.hpp>
#include <nebula_core_hw_interfaces/udp_packet_source.hpp>
#include <nebula_core_hw_interfaces/can_packet_source.hpp>

#include <memory>
#include <vector>

namespace nebula::drivers
{
struct LiveSessionConfig
{
  SensorModel model;
  SensorConfiguration sensor_config;
};

class LiveTransportGraph
{
public:
  LiveTransportGraph(std::shared_ptr<SensorRegistry> registry);
  
  void configure(const LiveSessionConfig & config);
  void set_output_callback(SensorOutputCallback callback);
  void set_progress_callback(SensorProgressCallback callback);
  
  void start();
  void stop();

private:
  void on_packet(const SensorPacket & packet);

  std::shared_ptr<SensorRegistry> registry_;
  std::unique_ptr<SensorDecoderRuntime> runtime_;
  std::vector<std::unique_ptr<PacketSource>> sources_;
  std::unique_ptr<PacketRouter> router_;
  
  SensorOutputCallback output_callback_;
  SensorProgressCallback progress_callback_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_LIVE_TRANSPORT_GRAPH_HPP
