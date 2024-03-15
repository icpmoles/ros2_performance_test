// Copyright 2022 Apex.AI, Inc.
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

#include <memory>

#include "performance_test/experiment_execution/runner_factory.hpp"

#include "performance_test/experiment_execution/inter_thread_runner.hpp"
#include "performance_test/experiment_execution/intra_thread_runner.hpp"
#include "performance_test/experiment_execution/round_trip_main_runner.hpp"
#include "performance_test/experiment_execution/round_trip_relay_runner.hpp"
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
#include "performance_test/experiment_execution/apex_os_runner.hpp"
#endif

namespace performance_test
{

std::unique_ptr<Runner> RunnerFactory::get(const ExperimentConfiguration & ec)
{
  if (ec.execution_strategy == "INTER_THREAD") {
    switch (ec.roundtrip_mode) {
      case RoundTripMode::NONE:
        return std::make_unique<InterThreadRunner>(ec);
      case RoundTripMode::MAIN:
        return std::make_unique<RoundTripMainRunner>(ec);
      case RoundTripMode::RELAY:
        return std::make_unique<RoundTripRelayRunner>(ec);
    }
  }
  if (ec.execution_strategy == "INTRA_THREAD") {
    return std::make_unique<IntraThreadRunner>(ec);
  }
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  if (ec.execution_strategy == "APEX_SINGLE_EXECUTOR") {
    if (ec.communicator == "ApexOSPollingSubscription") {
      return std::make_unique<ApexOsSingleExecutorRunner>(ec);
    }
  }
  if (ec.execution_strategy == "APEX_EXECUTOR_PER_COMMUNICATOR") {
    if (ec.communicator == "ApexOSPollingSubscription") {
      return std::make_unique<ApexOsExecutorPerCommunicatorRunner>(ec);
    }
  }
  if (ec.execution_strategy == "APEX_CHAIN") {
    if (ec.communicator == "ApexOSPollingSubscription") {
      return std::make_unique<ApexOsSingleExecutorChainRunner>(ec);
    }
  }
#endif
  throw std::invalid_argument("Invalid execution strategy!");
}

}  // namespace performance_test
