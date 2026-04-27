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

#ifndef NEBULA_PCAP_PACKET_SOURCE_HPP
#define NEBULA_PCAP_PACKET_SOURCE_HPP

#include <nebula_core_hw_interfaces/packet_source.hpp>

#include <string>
#include <thread>
#include <atomic>

namespace nebula::drivers
{
class PcapPacketSource : public PacketSource
{
public:
  PcapPacketSource();
  ~PcapPacketSource() override;

  void open(const std::string & pcap_file);
  void set_packet_callback(SensorPacketCallback callback) override;
  void start() override;
  void stop() override;

private:
  void run();

  std::string pcap_file_;
  SensorPacketCallback callback_;
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace nebula::drivers

#endif  // NEBULA_PCAP_PACKET_SOURCE_HPP
