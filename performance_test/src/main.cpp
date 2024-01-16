// Copyright 2017-2024 Apex.AI, Inc.
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

#include <iostream>

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
#include <rclcpp/rclcpp.hpp>
#endif

#include "performance_test/cli/cli_parser.hpp"
#include "performance_test/experiment_execution/runner_factory.hpp"
#include "performance_test/generated_messages/messages.hpp"
#include "performance_test/utilities/exit_request_handler.hpp"
#include "performance_test/utilities/prevent_cpu_idle.hpp"
#include "performance_test/utilities/rt_enabler.hpp"

int main(int argc, char ** argv)
{
  performance_test::CLIParser parser(argc, argv);

  if (parser.print_msg_list) {
    for (const auto & s : performance_test::messages::supported_msg_names()) {
      std::cout << s << std::endl;
    }
    return 0;
  }

  auto ec = parser.experiment_configuration;

  if (ec.rt_config.is_rt_init_required()) {
    performance_test::pre_proc_rt_init(ec.rt_config.cpus, ec.rt_config.prio);
  }

  if (ec.prevent_cpu_idle) {
    performance_test::prevent_cpu_idle();
  }

  performance_test::ExitRequestHandler::get().setup(ec.use_ros2_layers());

#if defined(PERFORMANCE_TEST_RCLCPP_ENABLED)
  // initialize ros
  if (ec.use_ros2_layers()) {
#ifdef APEX_CERT
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, false);
#else
    rclcpp::init(argc, argv);
#endif
  }
#endif

  auto r = performance_test::RunnerFactory::get(ec);

  if (ec.rt_config.is_rt_init_required()) {
    performance_test::post_proc_rt_init();
  }

  r->run();
}
