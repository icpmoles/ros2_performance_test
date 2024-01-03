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

#ifndef COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_

#include <cstdint>
#include <stdexcept>

#include "../experiment_metrics/message_received_listener.hpp"
#include "../utilities/msg_traits.hpp"
#include "../utilities/timestamp_provider.hpp"

namespace performance_test
{

class Publisher
{
public:
  virtual void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) = 0;

  virtual void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) = 0;

protected:
  template<typename T>
  inline void init_msg(
    T & msg,
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id)
  {
    MsgTraits::ensure_fixed_size(msg);
    msg.id = sample_id;
    msg.time = timestamp_provider.get();
  }
};

class Subscriber
{
public:
  virtual void update_subscription(MessageReceivedListener & listener) = 0;

  virtual void take(MessageReceivedListener &)
  {
    throw std::runtime_error("This communicator does not support take!");
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_
