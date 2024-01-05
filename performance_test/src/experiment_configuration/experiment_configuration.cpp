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

#include "performance_test/experiment_configuration/experiment_configuration.hpp"

#include <tclap/CmdLine.h>

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
#include <rmw/rmw.h>
#endif

#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
#include <settings/inspect.hpp>
#include <settings/repository.hpp>
#include <cyclone_dds_vendor/dds.hpp>
#endif

#include <csignal>
#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <vector>
#include <memory>
#include <sole/sole.hpp>

#include "performance_test/generated_messages/messages.hpp"
#include "performance_test/outputs/csv_output.hpp"
#include "performance_test/outputs/stdout_output.hpp"
#include "performance_test/outputs/json_output.hpp"
#include "performance_test/utilities/version.hpp"

namespace performance_test
{

std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e)
{
  if (e.is_setup()) {
    return stream <<
           "Experiment id: " << e.id() <<
           "\nPerformance Test Version: " << e.perf_test_version() <<
           "\nLogfile name: " << e.output_configuration().logfile_path <<
           "\nCommunication mean: " << e.com_mean() <<
           "\nRMW Implementation: " << e.rmw_implementation() <<
           "\nDDS domain id: " << e.dds_domain_id() <<
           "\nQOS: " << e.qos() <<
           "\nPublishing rate: " << e.rate() <<
           "\nTopic name: " << e.topic_name() <<
           "\nMsg name: " << e.msg_name() <<
           "\nMaximum runtime (sec): " << e.max_runtime() <<
           "\nNumber of publishers: " << e.number_of_publishers() <<
           "\nNumber of subscribers: " << e.number_of_subscribers() <<
           "\nMemory check enabled: " << e.check_memory() <<
           "\nWith security: " << e.is_with_security() <<
           "\nShared memory transfer: " << e.is_shared_memory_transfer() <<
           "\nZero copy transfer: " << e.is_zero_copy_transfer() <<
           "\nUnbounded message size: " << e.unbounded_msg_size() <<
           "\nRoundtrip Mode: " << e.roundtrip_mode() <<
           "\nIgnore seconds from beginning: " << e.rows_to_ignore();
  } else {
    return stream << "ERROR: Experiment is not yet setup!";
  }
}

ExperimentConfiguration::ExperimentConfiguration()
: m_id(sole::uuid4().str()),
  m_is_setup(false),
  m_dds_domain_id(),
  m_rate(),
  m_max_runtime(),
  m_rows_to_ignore(),
  m_number_of_publishers(),
  m_number_of_subscribers(),
  m_expected_num_pubs(),
  m_expected_num_subs(),
  m_wait_for_matched_timeout(),
  m_check_memory(false),
  m_is_zero_copy_transfer(false),
  m_prevent_cpu_idle(false),
  m_roundtrip_mode(RoundTripMode::NONE)
{
}

void ExperimentConfiguration::setup(int argc, char ** argv)
{
  bool print_msg_list = false;
  try {
    TCLAP::CmdLine cmd("Apex.AI performance_test");

    TCLAP::SwitchArg printToConsoleArg("", "print-to-console",
      "Print metrics to console.", cmd, false);

    TCLAP::ValueArg<std::string> LogfileArg("l", "logfile",
      "Specify the name of the log file, e.g. -l \"log_$(date +%F_%H-%M-%S).json\"."
      " Supported formats: csv, json", false, "", "name", cmd);

    TCLAP::ValueArg<uint32_t> rateArg("r", "rate",
      "The publishing rate. 0 means publish as fast as possible. "
      "Default is 1000.", false, 1000, "N", cmd);

    std::vector<std::string> allowedCommunications;

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
    allowedCommunications.push_back("rclcpp-single-threaded-executor");
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
    allowedCommunications.push_back("rclcpp-static-single-threaded-executor");
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
    allowedCommunications.push_back("rclcpp-waitset");
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    allowedCommunications.push_back("ApexOSPollingSubscription");
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
    allowedCommunications.push_back("FastRTPS");
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    allowedCommunications.push_back("ConnextDDSMicro");
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
    allowedCommunications.push_back("ConnextDDS");
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
    allowedCommunications.push_back("CycloneDDS");
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
    allowedCommunications.push_back("CycloneDDS-CXX");
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
    allowedCommunications.push_back("iceoryx");
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
    allowedCommunications.push_back("OpenDDS");
#endif
    TCLAP::ValuesConstraint<std::string> allowedCommunicationVals(allowedCommunications);
    TCLAP::ValueArg<std::string> communicationArg("c", "communication",
      "The communication plugin to use. "
      "Default is " + allowedCommunications[0] + ".", false, allowedCommunications[0],
      &allowedCommunicationVals, cmd);

    std::vector<std::string> allowedExecStrats;
    allowedExecStrats.push_back("INTER_THREAD");
    allowedExecStrats.push_back("INTRA_THREAD");
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    allowedExecStrats.push_back("APEX_SINGLE_EXECUTOR");
    allowedExecStrats.push_back("APEX_EXECUTOR_PER_COMMUNICATOR");
    allowedExecStrats.push_back("APEX_CHAIN");
#endif
    TCLAP::ValuesConstraint<std::string> allowedExecStratVals(allowedExecStrats);
    TCLAP::ValueArg<std::string> executionStrategyArg("e", "execution-strategy",
      "The execution strategy to use. "
      "Default is " + allowedExecStrats[0] + ".", false, allowedExecStrats[0],
      &allowedExecStratVals, cmd);

    TCLAP::ValueArg<std::string> topicArg("t", "topic", "The topic name. Default is test_topic.",
      false, "test_topic", "topic", cmd);

    std::vector<std::string> allowedMsgs = messages::supported_msg_names();
    TCLAP::ValuesConstraint<std::string> allowedMsgVals(allowedMsgs);
    TCLAP::ValueArg<std::string> msgArg("m", "msg",
      "The message type. Use --msg-list to list the options. "
      "Default is " + allowedMsgs[0] + ".", false, allowedMsgs[0], &allowedMsgVals, cmd);

    TCLAP::SwitchArg msgListArg("", "msg-list",
      "Print the list of available msg types and exit.", cmd, false);

    TCLAP::ValueArg<uint32_t> ddsDomainIdArg("", "dds-domain_id",
      "The DDS domain id. Default is 0.", false, 0, "id", cmd);

    std::vector<std::string> allowedReliabilityArgs{"RELIABLE", "BEST_EFFORT"};
    TCLAP::ValuesConstraint<std::string> allowedReliabilityArgsVals(allowedReliabilityArgs);
    TCLAP::ValueArg<std::string> reliabilityArg("", "reliability",
      "The QOS Reliability type. Default is BEST_EFFORT.", false, "BEST_EFFORT",
      &allowedReliabilityArgsVals, cmd);

    std::vector<std::string> allowedDurabilityArgs{"TRANSIENT_LOCAL", "VOLATILE"};
    TCLAP::ValuesConstraint<std::string> allowedDurabilityArgsVals(allowedDurabilityArgs);
    TCLAP::ValueArg<std::string> durabilityArg("", "durability",
      "The QOS Durability type. Default is VOLATILE.", false, "VOLATILE",
      &allowedDurabilityArgsVals, cmd);

    std::vector<std::string> allowedHistoryArgs{"KEEP_LAST", "KEEP_ALL"};
    TCLAP::ValuesConstraint<std::string> allowedHistoryArgsVals(allowedHistoryArgs);
    TCLAP::ValueArg<std::string> historyArg("", "history",
      "The QOS History type. Default is KEEP_ALL.", false, "KEEP_ALL",
      &allowedHistoryArgsVals, cmd);

    TCLAP::ValueArg<uint32_t> historyDepthArg("", "history-depth",
      "The history depth QOS. Default is 1000.", false, 1000, "N", cmd);

    TCLAP::ValueArg<uint64_t> maxRuntimeArg("", "max-runtime",
      "Run N seconds, then exit. 0 means run forever. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> numPubsArg("p", "num-pub-threads",
      "Number of publisher threads. Default is 1.", false, 1, "N", cmd);

    TCLAP::ValueArg<uint32_t> numSubsArg("s", "num-sub-threads",
      "Number of subscriber threads. Default is 1.", false, 1, "N", cmd);

    TCLAP::SwitchArg checkMemoryArg("", "check-memory",
      "Print backtrace of all memory operations performed by the middleware. "
      "This will slow down the application!", cmd, false);

    TCLAP::ValueArg<int32_t> useRtPrioArg("", "use-rt-prio",
      "Set RT priority using a SCHED_FIFO real-time policy. "
      "This option requires permissions to set a real-time priority. "
      "Default is 0 (disabled).",
      false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> useRtCpusArg("", "use-rt-cpus",
      "Set RT CPU affinity mask. "
      "The affinity mask has to be in decimal system. "
      "For example, 10 sets the affinity for processors 1 and 3. "
      "Default is 0 (disabled).",
      false, 0, "N", cmd);

    TCLAP::SwitchArg withSecurityArg("", "with-security",
      "Make nodes with deterministic names for use with security.", cmd, false);

    std::vector<std::string> allowedRoundTripModes{{"None", "Main", "Relay"}};
    TCLAP::ValuesConstraint<std::string> allowedRoundTripModeVals(allowedRoundTripModes);
    TCLAP::ValueArg<std::string> roundTripModeArg("", "roundtrip-mode",
      "Select the round trip mode. Default is None.", false, "None",
      &allowedRoundTripModeVals, cmd);

    TCLAP::ValueArg<uint32_t> ignoreArg("", "ignore",
      "Ignore the first N seconds of the experiment. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> expectedNumPubsArg("", "expected-num-pubs",
      "Expected number of publishers for wait-for-matched. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> expectedNumSubsArg("", "expected-num-subs",
      "Expected number of subscribers for wait-for-matched. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> waitForMatchedTimeoutArg("", "wait-for-matched-timeout",
      "Maximum time in seconds to wait for matched pubs/subs. Default is 30.", false, 30, "N", cmd);

    TCLAP::SwitchArg zeroCopyArg("", "zero-copy",
      "Use zero copy transfer.", cmd, false);

    TCLAP::ValueArg<uint32_t> unboundedMsgSizeArg("", "unbounded-msg-size",
      "The number of bytes to use for an unbounded message type. "
      "Ignored for other messages. Default is 0.",
      false, 0, "N", cmd);

    TCLAP::SwitchArg preventCpuIdleArg("", "prevent-cpu-idle",
      "Prevent CPU from entering idle states.", cmd, false);

    cmd.parse(argc, argv);

    m_rate = rateArg.getValue();
    m_com_mean = communication_mean_from_string(communicationArg.getValue());
    m_execution_strategy = execution_strategy_from_string(executionStrategyArg.getValue());
    m_topic_name = topicArg.getValue();
    m_msg_name = msgArg.getValue();
    print_msg_list = msgListArg.getValue();
    m_dds_domain_id = ddsDomainIdArg.getValue();
    m_qos.reliability = qos_reliability_from_string(reliabilityArg.getValue());
    m_qos.durability = qos_durability_from_string(durabilityArg.getValue());
    m_qos.history_kind = qos_history_kind_from_string(historyArg.getValue());
    m_qos.history_depth = historyDepthArg.getValue();
    m_max_runtime = maxRuntimeArg.getValue();
    m_number_of_publishers = numPubsArg.getValue();
    m_number_of_subscribers = numSubsArg.getValue();
    m_check_memory = checkMemoryArg.getValue();
    m_rt_config.prio = useRtPrioArg.getValue();
    m_rt_config.cpus = useRtCpusArg.getValue();
    m_with_security = withSecurityArg.getValue();
    m_roundtrip_mode = round_trip_mode_from_string(roundTripModeArg.getValue());
    m_rows_to_ignore = ignoreArg.getValue();
    m_expected_num_pubs = expectedNumPubsArg.getValue();
    m_expected_num_subs = expectedNumSubsArg.getValue();
    m_wait_for_matched_timeout = waitForMatchedTimeoutArg.getValue();
    m_is_zero_copy_transfer = zeroCopyArg.getValue();
    m_unbounded_msg_size = unboundedMsgSizeArg.getValue();
    m_prevent_cpu_idle = preventCpuIdleArg.getValue();
    m_output_configuration.print_to_console = printToConsoleArg.getValue();
    m_output_configuration.logfile_path = LogfileArg.getValue();
  } catch (TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  m_perf_test_version = version;

  try {
    if (print_msg_list) {
      for (const auto & s : messages::supported_msg_names()) {
        std::cout << s << std::endl;
      }
      // Exiting as we just print out some information and not running the
      // application.
      exit(0);
    }

    if (m_number_of_publishers > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    if (m_expected_num_pubs > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    if (m_with_security) {
      if (!use_ros2_layers()) {
        throw std::invalid_argument("Only ROS2 supports security!");
      }
    }

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
    m_rmw_implementation = rmw_get_implementation_identifier();
#else
    m_rmw_implementation = "N/A";
#endif
    m_is_setup = true;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: ";
    std::cerr << e.what() << std::endl;
    exit(1);
  }
}

bool ExperimentConfiguration::is_setup() const
{
  return m_is_setup;
}
CommunicationMean ExperimentConfiguration::com_mean() const
{
  check_setup();
  return m_com_mean;
}
ExecutionStrategy ExperimentConfiguration::execution_strategy() const
{
  check_setup();
  return m_execution_strategy;
}
bool ExperimentConfiguration::use_ros2_layers() const
{
#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR) {
    return true;
  }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR) {
    return true;
  }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_WAITSET) {
    return true;
  }
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  if (com_mean() == CommunicationMean::ApexOSPollingSubscription) {
    return true;
  }
#endif
  return false;
}
uint32_t ExperimentConfiguration::dds_domain_id() const
{
  check_setup();
  return m_dds_domain_id;
}
QOSAbstraction ExperimentConfiguration::qos() const
{
  check_setup();
  return m_qos;
}
uint32_t ExperimentConfiguration::rate() const
{
  check_setup();
  return m_rate;
}
std::chrono::duration<double> ExperimentConfiguration::period() const
{
  check_setup();
  return std::chrono::duration<double>(1.0 / rate());
}
std::chrono::nanoseconds ExperimentConfiguration::period_ns() const
{
  check_setup();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period());
}
std::string ExperimentConfiguration::topic_name() const
{
  check_setup();
  return m_topic_name;
}
std::string ExperimentConfiguration::msg_name() const
{
  check_setup();
  return m_msg_name;
}
uint64_t ExperimentConfiguration::max_runtime() const
{
  check_setup();
  return m_max_runtime;
}
uint32_t ExperimentConfiguration::rows_to_ignore() const
{
  check_setup();
  return m_rows_to_ignore;
}
uint32_t ExperimentConfiguration::number_of_publishers() const
{
  check_setup();
  return m_number_of_publishers;
}
uint32_t ExperimentConfiguration::number_of_subscribers() const
{
  check_setup();
  return m_number_of_subscribers;
}

uint32_t ExperimentConfiguration::expected_num_pubs() const
{
  check_setup();
  return m_expected_num_pubs;
}
uint32_t ExperimentConfiguration::expected_num_subs() const
{
  check_setup();
  return m_expected_num_subs;
}

std::chrono::seconds ExperimentConfiguration::expected_wait_for_matched_timeout() const
{
  check_setup();
  return std::chrono::seconds(m_wait_for_matched_timeout);
}

bool ExperimentConfiguration::check_memory() const
{
  check_setup();
  return m_check_memory;
}

RealTimeConfiguration ExperimentConfiguration::rt_config() const
{
  check_setup();
  return m_rt_config;
}

bool ExperimentConfiguration::is_with_security() const
{
  check_setup();
  return m_with_security;
}

bool ExperimentConfiguration::is_shared_memory_transfer() const
{
  check_setup();
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
#ifdef DDSCXX_HAS_SHM
  if (rmw_implementation() == "rmw_apex_middleware" && use_ros2_layers()) {
    return apex::settings::inspect::get_or_default<bool>(
      apex::settings::repository::get(), "domain/shared_memory/enable", false);
  }
#endif
#endif
  return false;
}

bool ExperimentConfiguration::is_zero_copy_transfer() const
{
  check_setup();
  return m_is_zero_copy_transfer;
}

RoundTripMode ExperimentConfiguration::roundtrip_mode() const
{
  check_setup();
  return m_roundtrip_mode;
}

std::string ExperimentConfiguration::rmw_implementation() const
{
  check_setup();
  return m_rmw_implementation;
}

std::string ExperimentConfiguration::perf_test_version() const
{
  return m_perf_test_version;
}

std::string ExperimentConfiguration::pub_topic_postfix() const
{
  check_setup();
  std::string fix;
  if (m_roundtrip_mode == RoundTripMode::MAIN) {
    fix = "main";
  } else if (m_roundtrip_mode == RoundTripMode::RELAY) {
    fix = "relay";
  }
  return fix;
}

std::string ExperimentConfiguration::sub_topic_postfix() const
{
  check_setup();
  std::string fix;
  if (m_roundtrip_mode == RoundTripMode::MAIN) {
    fix = "relay";
  } else if (m_roundtrip_mode == RoundTripMode::RELAY) {
    fix = "main";
  }
  return fix;
}

std::string ExperimentConfiguration::id() const
{
  return m_id;
}

size_t ExperimentConfiguration::unbounded_msg_size() const
{
  check_setup();
  return m_unbounded_msg_size;
}

void ExperimentConfiguration::check_setup() const
{
  if (!m_is_setup) {
    throw std::runtime_error("Experiment is not yet setup!");
  }
}

bool ExperimentConfiguration::prevent_cpu_idle() const
{
  return m_prevent_cpu_idle;
}

OutputConfiguration ExperimentConfiguration::output_configuration() const
{
  return m_output_configuration;
}

}  // namespace performance_test
