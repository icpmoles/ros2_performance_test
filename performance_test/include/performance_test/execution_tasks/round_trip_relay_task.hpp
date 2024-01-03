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

#ifndef EXECUTION_TASKS__ROUND_TRIP_RELAY_TASK_HPP_
#define EXECUTION_TASKS__ROUND_TRIP_RELAY_TASK_HPP_

#include <memory>

#include "performance_test/communication_abstractions/communicator.hpp"
#include "performance_test/experiment_metrics/message_received_listener.hpp"
#include "performance_test/utilities/memory_checker.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{
class RoundTripRelayTask : public MessageReceivedListener
{
public:
  RoundTripRelayTask(
    const ExperimentConfiguration & ec,
    std::shared_ptr<Publisher> pub,
    std::shared_ptr<Subscriber> sub)
  : m_pub(pub),
    m_sub(sub),
    m_memory_checker(ec) {}

  RoundTripRelayTask & operator=(const RoundTripRelayTask &) = delete;
  RoundTripRelayTask(const RoundTripRelayTask &) = delete;

  void run()
  {
    m_sub->update_subscription(*this);
    m_memory_checker.enable_memory_tools_checker();
  }

  void on_message_received(
    const std::int64_t time_msg_sent_ns,
    const std::int64_t,
    const std::uint64_t sample_id,
    const std::size_t
  ) override
  {
    RoundtripTimestampProvider ts(time_msg_sent_ns);
    m_pub->publish_copy(ts, sample_id);
  }

private:
  std::shared_ptr<Publisher> m_pub;
  std::shared_ptr<Subscriber> m_sub;
  MemoryChecker m_memory_checker;
};

}  // namespace performance_test

#endif  // EXECUTION_TASKS__ROUND_TRIP_RELAY_TASK_HPP_
