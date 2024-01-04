// Copyright 2017 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__EXECUTION_TASKS__PUBLISHER_TASK_HPP_
#define PERFORMANCE_TEST__EXECUTION_TASKS__PUBLISHER_TASK_HPP_

#include <memory>
#include <thread>
#include <functional>

#include "performance_test/experiment_metrics/publisher_stats.hpp"
#include "performance_test/utilities/memory_checker.hpp"
#include "performance_test/utilities/perf_clock.hpp"
#include "performance_test/utilities/spin_lock.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{
class PublisherTask
{
public:
  PublisherTask(
    const ExperimentConfiguration & ec,
    PublisherStats & stats,
    std::shared_ptr<Publisher> pub)
  : m_ec(ec),
    m_stats(stats),
    m_pub(pub),
    m_time_between_publish(ec.period()),
    m_first_run(perf_clock::now()),
    m_next_run(perf_clock::now() + ec.period_ns()),
    m_memory_checker(ec) {}

  PublisherTask & operator=(const PublisherTask &) = delete;
  PublisherTask(const PublisherTask &) = delete;

  void run()
  {
    // We track here how much time (can also be negative) was left for the
    // loop iteration given the desired loop rate.
    const std::chrono::nanoseconds reserve = m_next_run - perf_clock::now();

    if (reserve.count() > 0 &&
      m_ec.roundtrip_mode() != RoundTripMode::RELAY)
    {
      std::this_thread::sleep_until(m_next_run);
    }

    if (m_ec.is_zero_copy_transfer()) {
      m_pub->publish_loaned(m_timestamp_provider, m_stats.next_sample_id());
    } else {
      m_pub->publish_copy(m_timestamp_provider, m_stats.next_sample_id());
    }
    m_stats.on_message_sent();

    m_next_run =
      m_first_run +
      m_loop_counter * std::chrono::duration_cast<std::chrono::nanoseconds>(
      m_time_between_publish);
    ++m_loop_counter;
    m_memory_checker.enable_memory_tools_checker();
  }

  const ExperimentConfiguration & m_ec;
  PublisherStats & m_stats;
  std::shared_ptr<Publisher> m_pub;
  const std::chrono::duration<double> m_time_between_publish;
  const perf_clock::time_point m_first_run;
  perf_clock::time_point m_next_run;
  std::size_t m_loop_counter{1};
  MemoryChecker m_memory_checker;
  PublisherTimestampProvider m_timestamp_provider;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXECUTION_TASKS__PUBLISHER_TASK_HPP_
