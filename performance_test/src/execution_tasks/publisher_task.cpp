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

#include "performance_test/execution_tasks/publisher_task.hpp"

#include <memory>
#include <thread>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_metrics/publisher_stats.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/utilities/memory_checker.hpp"
#include "performance_test/utilities/perf_clock.hpp"

namespace performance_test
{

PublisherTask::PublisherTask(
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

void PublisherTask::run()
{
  // We track here how much time (can also be negative) was left for the
  // loop iteration given the desired loop rate.
  const std::chrono::nanoseconds reserve = m_next_run - perf_clock::now();

  if (reserve.count() > 0) {
    std::this_thread::sleep_until(m_next_run);
  }

  if (m_ec.is_zero_copy_transfer) {
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

}  // namespace performance_test
