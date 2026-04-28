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

#pragma once

#include <nebula_core_common/radar_types.hpp>
#include <nebula_continental_common/continental_ars548.hpp>
#include <nebula_continental_common/continental_srr520.hpp>

#include <memory>
#include <vector>
#include <functional>

namespace nebula::drivers {

class AgnosticARS548Decoder {
public:
  explicit AgnosticARS548Decoder(const std::shared_ptr<const continental_ars548::ContinentalARS548SensorConfiguration>& config);
  ~AgnosticARS548Decoder();
  bool process_packet(const std::vector<uint8_t>& packet_data, uint64_t timestamp_ns);
  void set_detection_callback(std::function<void(std::shared_ptr<RadarDetectionList>)> cb);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class AgnosticSRR520Decoder {
public:
  explicit AgnosticSRR520Decoder(const std::shared_ptr<const continental_srr520::ContinentalSRR520SensorConfiguration>& config);
  ~AgnosticSRR520Decoder();
  bool process_packet(const std::vector<uint8_t>& packet_data, uint64_t timestamp_ns);
  void set_detection_callback(std::function<void(std::shared_ptr<RadarDetectionList>)> cb);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nebula::drivers
