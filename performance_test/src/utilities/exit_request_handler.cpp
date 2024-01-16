// Copyright 2024 Apex.AI, Inc.
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

#include "performance_test/utilities/exit_request_handler.hpp"

#include <csignal>

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
#include <rclcpp/rclcpp.hpp>
#endif

static void handle_sigint(int sig)
{
  std::signal(sig, SIG_DFL);
  performance_test::ExitRequestHandler::get().request_exit();
}

namespace performance_test
{

ExitRequestHandler::ExitRequestHandler()
: m_exit_requested(false), m_use_ros2_layers(false) {}

void ExitRequestHandler::setup(bool use_ros2_layers)
{
  m_use_ros2_layers = use_ros2_layers;
  if (!use_ros2_layers) {
    std::signal(SIGINT, handle_sigint);
  }
}

bool ExitRequestHandler::exit_requested() const
{
  bool ret_val = m_exit_requested;
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  ret_val |= m_use_ros2_layers && !rclcpp::ok();
#endif
  return ret_val;
}

void ExitRequestHandler::request_exit()
{
  m_exit_requested = true;
}

}  // namespace performance_test
