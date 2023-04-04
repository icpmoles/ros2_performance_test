// Copyright 2021 Apex.AI, Inc.
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

#include "json_output.hpp"

#include <string>
#include <iomanip>

#include "../experiment_metrics/analysis_result.hpp"

namespace performance_test
{
JsonOutput::JsonOutput()
: m_ec(ExperimentConfiguration::get()), m_sb(), m_writer(m_sb) {}

JsonOutput::~JsonOutput()
{
  close();
}

void JsonOutput::open()
{
  if (m_ec.is_setup() && !m_ec.logfile_name().empty()) {
    m_os.open(m_ec.logfile_name(), std::ofstream::out);

    std::cout << "Writing JSON output to: " << m_ec.logfile_name() << std::endl;

    m_writer.StartObject();
    write(m_ec);

    m_writer.String("analysis_results");
    m_writer.StartArray();
  }
}
void JsonOutput::update(const AnalysisResult & result)
{
  write(result);
}

void JsonOutput::close()
{
  if (m_os.is_open()) {
    m_writer.EndArray();
    m_writer.EndObject();

    m_os << m_sb.GetString();
    m_os.close();
  }
}

void JsonOutput::write(const ExperimentConfiguration & ec)
{
  write("id", ec.id());
  write("perf_test_version", ec.perf_test_version());
  write("final_logfile_name", tableau_final_logfile_name(ec.id(), ec.topic_name()));
  write("com_mean_str", to_string(ec.com_mean()));
  write("rmw_implementation", ec.rmw_implementation());
  write("dds_domain_id", ec.dds_domain_id());
  write("qos_reliability", to_string(ec.qos().reliability));
  write("qos_durability", to_string(ec.qos().durability));
  write("qos_history_kind", to_string(ec.qos().history_kind));
  write("qos_history_depth", ec.qos().history_depth);
  write("rate", ec.rate());
  write("topic_name", ec.topic_name());
  write("msg_name", ec.msg_name());
  write("max_runtime", ec.max_runtime());
  write("number_of_publishers", ec.number_of_publishers());
  write("number_of_subscribers", ec.number_of_subscribers());
  write("check_memory", ec.check_memory());
  // TODO(erik.snider) delete when gc parse_upload is fixed
  write("use_single_participant", false);
  write("with_security", ec.is_with_security());
  write("is_zero_copy_transfer", ec.is_zero_copy_transfer());
  write("roundtrip_mode", to_string(ec.roundtrip_mode()));
  write("is_rt_init_required", ec.is_rt_init_required());
  write("external_info_githash", ec.get_external_info().m_githash);
  write("external_info_platform", ec.get_external_info().m_platform);
  write("external_info_branch", ec.get_external_info().m_branch);
  write("external_info_architecture", ec.get_external_info().m_architecture);
  write("external_info_ci", ec.get_external_info().m_ci);
}

void JsonOutput::write(const AnalysisResult & ar)
{
  m_writer.StartObject();
  write("experiment_start", ar.m_experiment_start);
  write("loop_start", ar.m_time_between_two_measurements);
  write("num_samples_received", ar.m_num_samples_received);
  write("num_samples_sent", ar.m_num_samples_sent);
  write("num_samples_lost", ar.m_num_samples_lost);
  write("total_data_received", ar.m_total_data_received);
  write("latency_min", ar.m_latency.min());
  write("latency_max", ar.m_latency.max());
  write("latency_n", ar.m_latency.n());
  write("latency_mean", ar.m_latency.mean());
  write("latency_M2", ar.m_latency.m2());
  write("latency_variance", ar.m_latency.variance());
#if !defined(WIN32)
  write("sys_tracker_ru_utime", ar.m_sys_usage.ru_utime);
  write("sys_tracker_ru_stime", ar.m_sys_usage.ru_stime);
  write("sys_tracker_ru_maxrss", ar.m_sys_usage.ru_maxrss);
  write("sys_tracker_ru_ixrss", ar.m_sys_usage.ru_ixrss);
  write("sys_tracker_ru_idrss", ar.m_sys_usage.ru_idrss);
  write("sys_tracker_ru_isrss", ar.m_sys_usage.ru_isrss);
  write("sys_tracker_ru_minflt", ar.m_sys_usage.ru_minflt);
  write("sys_tracker_ru_majflt", ar.m_sys_usage.ru_majflt);
  write("sys_tracker_ru_nswap", ar.m_sys_usage.ru_nswap);
  write("sys_tracker_ru_inblock", ar.m_sys_usage.ru_inblock);
  write("sys_tracker_ru_oublock", ar.m_sys_usage.ru_oublock);
  write("sys_tracker_ru_msgsnd", ar.m_sys_usage.ru_msgsnd);
  write("sys_tracker_ru_msgrcv", ar.m_sys_usage.ru_msgrcv);
  write("sys_tracker_ru_nsignals", ar.m_sys_usage.ru_nsignals);
  write("sys_tracker_ru_nvcsw", ar.m_sys_usage.ru_nvcsw);
  write("sys_tracker_ru_nivcsw", ar.m_sys_usage.ru_nivcsw);
#endif
  write("cpu_info_cpu_cores", ar.m_cpu_info.cpu_cores());
  write("cpu_info_cpu_usage", ar.m_cpu_info.cpu_usage());
  m_writer.EndObject();
}

// Tableau parses the date and time out of the final_logfile_name column.
// This workaround feel bad.
// It would be much better if there were a dedicated datetime column.
std::string JsonOutput::tableau_final_logfile_name(
  const std::string & id, const std::string & topic)
{
  auto t = std::time(nullptr);
  auto tm = *std::gmtime(&t);
  std::ostringstream oss;
  oss << id << "_" << topic << std::put_time(&tm, "_%d-%m-%Y_%H-%M-%S");
  return oss.str();
}

void JsonOutput::write(const char * key, const std::string & val)
{
  m_writer.String(key);
  m_writer.String(val.c_str());
}

void JsonOutput::write(const char * key, uint32_t val)
{
  m_writer.String(key);
  m_writer.Uint(val);
}

void JsonOutput::write(const char * key, uint64_t val)
{
  m_writer.String(key);
  m_writer.Uint64(val);
}

void JsonOutput::write(const char * key, int32_t val)
{
  m_writer.String(key);
  m_writer.Int(val);
}

void JsonOutput::write(const char * key, int64_t val)
{
  m_writer.String(key);
  m_writer.Int64(val);
}

void JsonOutput::write(const char * key, float val)
{
  m_writer.String(key);
  if (std::isfinite(val)) {
    m_writer.Double(val);
  } else {
    m_writer.Double(0.0);
  }
}

void JsonOutput::write(const char * key, double val)
{
  m_writer.String(key);
  if (std::isfinite(val)) {
    m_writer.Double(val);
  } else {
    m_writer.Double(0.0);
  }
}

void JsonOutput::write(const char * key, bool val)
{
  m_writer.String(key);
  m_writer.Bool(val);
}

#if !defined(WIN32)
void JsonOutput::write(const char * key, timeval val)
{
  m_writer.String(key);
  std::chrono::nanoseconds ns =
    std::chrono::seconds(val.tv_sec) +
    std::chrono::microseconds(val.tv_usec);
  m_writer.Int64(ns.count());
}
#endif

void JsonOutput::write(const char * key, const std::chrono::nanoseconds val)
{
  m_writer.String(key);
  m_writer.Int64(val.count());
}

}  // namespace performance_test
