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

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  #include <rti_me_cpp.hxx>
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  #include <ndds/ndds_cpp.h>
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  #include <dds/dds.h>
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
  #include <dds/dds.hpp>
#endif

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

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  DDSDomainParticipant * connext_DDS_micro_participant(
    const ExperimentConfiguration & ec) const;

  /**
   * \brief Creates a new Connext DDS Micro publisher.
   * \param publisher Will be overwritten with the created publisher.
   * \param dw_qos Will be overwritten with the default QOS from the created publisher.
   */
  void connext_dds_micro_publisher(
    const ExperimentConfiguration & ec,
    DDSPublisher * & publisher,
    DDS_DataWriterQos & dw_qos) const;

  /**
   * \brief Creates a new Connext DDS Micro subscriber.
   * \param subscriber Will be overwritten with the created subscriber.
   * \param dr_qos Will be overwritten with the default QOS from the created subscriber.
   */
  void connext_dds_micro_subscriber(
    const ExperimentConfiguration & ec,
    DDSSubscriber * & subscriber,
    DDS_DataReaderQos & dr_qos) const;
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  DDSDomainParticipant * connext_dds_participant(const ExperimentConfiguration & ec) const;

  /**
   * \brief Creates a new Connext DDS publisher.
   * \param publisher Will be overwritten with the created publisher.
   * \param dw_qos Will be overwritten with the default QOS from the created publisher.
   */
  void connext_dds_publisher(
    const ExperimentConfiguration & ec,
    DDSPublisher * & publisher,
    DDS_DataWriterQos & dw_qos) const;

  /**
   * \brief Creates a new Connext DDS subscriber.
   * \param subscriber Will be overwritten with the created subscriber.
   * \param dr_qos Will be overwritten with the default QOS from the created subscriber.
   */
  void connext_dds_subscriber(
    const ExperimentConfiguration & ec,
    DDSSubscriber * & subscriber,
    DDS_DataReaderQos & dr_qos) const;
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  dds_entity_t cyclonedds_participant(const ExperimentConfiguration & ec) const;
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
  dds::domain::DomainParticipant cyclonedds_cxx_participant(
    const ExperimentConfiguration & ec) const;
#endif

private:
  ResourceManager()
  : m_unused(nullptr)
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
    , m_node(nullptr)
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    , m_connext_dds_micro_participant(nullptr)
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
    , m_connext_dds_participant(nullptr)
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
    , m_cyclonedds_participant(0)
#endif
  {}

  const void * m_unused;

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  mutable std::shared_ptr<rclcpp::Node> m_node;
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  mutable DDSDomainParticipant * m_connext_dds_micro_participant;
  mutable NETIO_SHMEMInterfaceFactoryProperty m_shmem_property;
  mutable DPDE_DiscoveryPluginProperty m_dpde_property;
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  mutable DDSDomainParticipant * m_connext_dds_participant;
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  mutable dds_entity_t m_cyclonedds_participant;
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
  mutable dds::domain::DomainParticipant m_cyclonedds_cxx_participant{dds::core::null};
#endif

  mutable std::mutex m_global_mutex;
};

}  // namespace performance_test
#endif  // PERFORMANCE_TEST__COMMUNICATION_ABSTRACTIONS__RESOURCE_MANAGER_HPP_
