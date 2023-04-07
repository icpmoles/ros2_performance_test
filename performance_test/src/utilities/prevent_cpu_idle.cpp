// Copyright 2023 Apex.AI, Inc.
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

#include "prevent_cpu_idle.hpp"

#include <memory>
#include <stdexcept>
#include <iostream>

namespace performance_test
{
void prevent_cpu_idle()
{
#ifdef PERFORMANCE_TEST_LINUX
  static std::unique_ptr<FILE, int (*)(FILE *)> cpu_dma_latency
  {
    ::fopen("/dev/cpu_dma_latency", "wb"),
    ::fclose
  };

  if (!cpu_dma_latency) {
    if (errno == ENOENT) {
      throw std::runtime_error("The file /dev/cpu_dma_latency does not exist");
    }
    throw std::runtime_error("Failed to open /dev/cpu_dma_latency");
  }

  ::setbuf(cpu_dma_latency.get(), nullptr);

  std::int32_t latency_i32 = 0;
  if (::fwrite(&latency_i32, sizeof(latency_i32), 1, cpu_dma_latency.get()) != 1) {
    throw std::runtime_error("Failed to write latency specification to /dev/cpu_dma_latency");
  }
#else
  throw std::runtime_error("prevent_cpu_idle is not supported on this platform");
#endif
}
}  // namespace performance_test
