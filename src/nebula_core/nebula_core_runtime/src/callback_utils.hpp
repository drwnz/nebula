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

#ifndef NEBULA_RUNTIME_CALLBACK_UTILS_HPP
#define NEBULA_RUNTIME_CALLBACK_UTILS_HPP

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>

namespace nebula::drivers
{
inline bool should_log_user_callback_exception()
{
  static std::atomic<uint32_t> log_count{0};
  constexpr uint32_t k_max_callback_exception_logs = 16;
  return log_count.fetch_add(1, std::memory_order_relaxed) < k_max_callback_exception_logs;
}

template <typename CallbackT, typename ArgT>
void invoke_user_callback(
  const char * owner_name, const char * callback_name, const CallbackT & callback, const ArgT & arg)
{
  if (!callback) {
    return;
  }

  try {
    callback(arg);
  } catch (const std::exception & e) {
    if (should_log_user_callback_exception()) {
      std::cerr << owner_name << ": " << callback_name << " callback threw: " << e.what()
                << std::endl;
    }
  } catch (...) {
    if (should_log_user_callback_exception()) {
      std::cerr << owner_name << ": " << callback_name << " callback threw a non-std::exception"
                << std::endl;
    }
  }
}

}  // namespace nebula::drivers

#endif  // NEBULA_RUNTIME_CALLBACK_UTILS_HPP
