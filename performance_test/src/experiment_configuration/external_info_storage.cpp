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

#include "performance_test/experiment_configuration/external_info_storage.hpp"

#include <rapidjson/document.h>
#include <string>

namespace performance_test
{

ExternalInfoStorage::ExternalInfoStorage()
{
  const auto ptr = std::getenv("APEX_PERFORMANCE_TEST");
  if (ptr) {
    rapidjson::Document document;
    document.Parse(ptr);

    for (auto & m : document.GetObject()) {
      m_to_log = m_to_log + m.name.GetString() + ": " + m.value.GetString() + "\n";
      m_external_info[m.name.GetString()] = m.value.GetString();
    }
  }
}
}  // namespace performance_test
