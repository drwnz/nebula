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

#include <nebula_core_hw_interfaces/tcp_packet_source.hpp>
#include <iostream>
#include <chrono>

namespace nebula::drivers
{

TcpPacketSource::TcpPacketSource() {}

TcpPacketSource::~TcpPacketSource()
{
  stop();
}

void TcpPacketSource::configure(const std::string & host_ip, uint16_t port)
{
  host_ip_ = host_ip;
  port_ = port;
}

void TcpPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void TcpPacketSource::start()
{
  if (running_) return;
  running_ = true;
  thread_ = std::thread(&TcpPacketSource::run, this);
}

void TcpPacketSource::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void TcpPacketSource::run()
{
  try {
      auto s_builder = connections::TcpSocket::Builder(host_ip_, port_);
      s_builder.set_connect_timeout(1000);
      auto sock = std::make_unique<connections::TcpSocket>(std::move(s_builder).connect());
      socket_ = std::move(sock);
  } catch (const std::exception & e) {
      std::cerr << "TcpPacketSource failed to connect to " << host_ip_ << ":" << port_ << " - " << e.what() << std::endl;
      running_ = false;
      return;
  }

  while (running_ && socket_) {
      try {
          auto data = socket_->receive(std::chrono::milliseconds(100));
          if (!data.empty() && callback_) {
              SensorPacket sp;
              sp.transport = SensorTransportKind::TCP;
              sp.channel = SensorPacketChannel::Unknown;
              sp.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
              
              SensorEndpoint dst;
              dst.address = host_ip_;
              dst.port = port_;
              sp.destination = dst;
              sp.payload = std::move(data);
              
              callback_(sp);
          }
      } catch (const std::exception & e) {
          std::cerr << "TcpPacketSource receive error: " << e.what() << std::endl;
          // Connection likely closed or failed. 
          break;
      }
  }

  socket_.reset();
  running_ = false;
}

}  // namespace nebula::drivers
