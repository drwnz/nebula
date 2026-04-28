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

#include <nebula_core_runtime/live_transport_graph.hpp>

#include <iostream>

namespace nebula::drivers
{

LiveTransportGraph::LiveTransportGraph(std::shared_ptr<SensorRegistry> registry)
: registry_(registry)
{
}

void LiveTransportGraph::configure(const LiveSessionConfig & config)
{
  auto metadata = registry_->find_plugin_for_model(config.model);
  if (!metadata) {
    throw std::runtime_error("No plugin found for model");
  }

  auto plugin = registry_->load_plugin(*metadata);
  if (!plugin) {
    throw std::runtime_error("Failed to load plugin");
  }

  runtime_ = plugin->create_decoder_runtime();
  runtime_->configure(config.sensor_config);
  
  if (output_callback_) {
    runtime_->set_output_callback(output_callback_);
  }
  if (progress_callback_) {
    runtime_->set_progress_callback(progress_callback_);
  }

  router_ = std::make_unique<PacketRouter>();
  router_->configure(plugin->packet_requirements(config.sensor_config));

  sources_.clear();
  auto requirements = plugin->live_transport_requirements(config.sensor_config);
  for (const auto & req : requirements) {
      if (req.transport == SensorTransportKind::UDP && req.port.has_value()) {
          auto source = std::make_unique<UdpPacketSource>();
          source->configure(config.sensor_config.host_ip, *req.port);
          source->set_packet_callback(std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
          sources_.push_back(std::move(source));
      } else if (req.transport == SensorTransportKind::CAN) {
          auto source = std::make_unique<CanPacketSource>();
          std::string interface = "can0";
          if (config.extra_params.count("can_interface")) {
              interface = config.extra_params.at("can_interface");
          }
          source->configure(interface); 
          source->set_packet_callback(std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
          sources_.push_back(std::move(source));
      } else if (req.transport == SensorTransportKind::TCP && req.port.has_value()) {
          auto source = std::make_unique<TcpPacketSource>();
          source->configure(config.sensor_config.host_ip, *req.port);
          source->set_packet_callback(std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
          sources_.push_back(std::move(source));
      } else if (req.transport == SensorTransportKind::HTTP && req.port.has_value()) {
          auto source = std::make_unique<HttpPacketSource>();
          std::string path = "/";
          if (req.http_path.has_value()) path = *req.http_path;
          source->configure(config.sensor_config.host_ip, *req.port, path);
          source->set_packet_callback(std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
          sources_.push_back(std::move(source));
      }
  }
}

void LiveTransportGraph::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
  if (runtime_) {
    runtime_->set_output_callback(output_callback_);
  }
}

void LiveTransportGraph::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
  if (runtime_) {
    runtime_->set_progress_callback(progress_callback_);
  }
}

void LiveTransportGraph::start()
{
  for (auto & source : sources_) {
    source->start();
  }
}

void LiveTransportGraph::stop()
{
  for (auto & source : sources_) {
    source->stop();
  }
}

void LiveTransportGraph::on_packet(const SensorPacket & packet)
{
  std::lock_guard<std::mutex> lock(mutex_);
  SensorPacket mutable_packet = packet;
  if (router_->route(mutable_packet)) {
    runtime_->process_packet(mutable_packet);
  }
}

}  // namespace nebula::drivers
