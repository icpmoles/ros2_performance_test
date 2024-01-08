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

#ifndef PERFORMANCE_TEST__COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_
#define PERFORMANCE_TEST__COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  #include <rclcpp/rclcpp.hpp>
#endif

#include <cstdlib>
#include <memory>
#include <mutex>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"

namespace performance_test
{

/// Stores and manages global resources for the communication plugins.
class ResourceManager
{
public:
  // Standard C++11 singleton pattern.
  /// Singleton instance getter.
  static ResourceManager & get()
  {
    static ResourceManager instance;

    return instance;
  }

  static void shutdown();

  ResourceManager(ResourceManager const &) = delete;
  ResourceManager(ResourceManager &&) = delete;
  ResourceManager & operator=(ResourceManager const &) = delete;
  ResourceManager & operator=(ResourceManager &&) = delete;

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  std::shared_ptr<rclcpp::Node> rclcpp_node(const ExperimentConfiguration & ec) const;
#endif

private:
  ResourceManager()
  : m_unused(nullptr)
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
    , m_node(nullptr)
#endif
  {}

  const void * m_unused;

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  mutable std::shared_ptr<rclcpp::Node> m_node;
#endif

  mutable std::mutex m_global_mutex;
};

}  // namespace performance_test
#endif  // PERFORMANCE_TEST__COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_
