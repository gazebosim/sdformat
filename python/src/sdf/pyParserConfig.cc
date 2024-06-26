/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pyParserConfig.hh"

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "sdf/ParserConfig.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineParserConfig(pybind11::object module)
{
  pybind11::class_<sdf::ParserConfig> parseConfigModule(module, "ParserConfig");
  parseConfigModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::ParserConfig>())
    .def("global_config",
         &sdf::ParserConfig::GlobalConfig,
         "Mutable access to a singleton ParserConfig that serves as the global "
         "ParserConfig object for all parsing operations that do not specify "
         "their own ParserConfig")
    .def("find_file_callback",
         &sdf::ParserConfig::FindFileCallback,
         "Get the find file callback function")
    .def("calculate_inertial_configuration",
         &sdf::ParserConfig::CalculateInertialConfiguration,
         "Get the current configuration for the CalculateInertial() function")
    .def("set_calculate_inertial_configuration",
         &sdf::ParserConfig::SetCalculateInertialConfiguration,
         "Set the configuration for the CalculateInertial() function")
    .def("set_find_callback",
         &sdf::ParserConfig::SetFindCallback,
         "Set the callback to use when libsdformat can't find a file.")
    .def("uri_path_map",
         &sdf::ParserConfig::URIPathMap,
         "Get the URI scheme to search directories map")
    .def("add_uri_path",
         &sdf::ParserConfig::AddURIPath,
         "Associate paths to a URI.")
    .def("set_warnings_policy",
         &sdf::ParserConfig::SetWarningsPolicy,
         "Set the warning enforcment policy.")
    .def("warnings_policy",
         &sdf::ParserConfig::WarningsPolicy,
         "Get the current warning enforcement policy")
    .def("set_unrecognized_elements_policy",
         &sdf::ParserConfig::SetUnrecognizedElementsPolicy,
         "Set the policy for unrecogonized elements without an xmlns")
    .def("unrecognized_elements_policy",
         &sdf::ParserConfig::UnrecognizedElementsPolicy,
         "Get the current unrecognized elements policy")
    .def("set_deprecated_elements_policy",
         &sdf::ParserConfig::SetDeprecatedElementsPolicy,
         "Set the policy for deprecated elements.")
    .def("reset_deprecated_elements_policy",
         &sdf::ParserConfig::ResetDeprecatedElementsPolicy,
         "Resets the policy for deprecated elements so that it follows "
         "WarningsPolicy.")
    .def("deprecated_elements_policy",
         &sdf::ParserConfig::DeprecatedElementsPolicy,
         "Get the current deprecated elements policy")
    .def("urdf_set_preserve_fixed_joint",
         &sdf::ParserConfig::URDFSetPreserveFixedJoint,
         "Set the preserveFixedJoint flag.")
    .def("urdf_preserve_fixed_joint",
         &sdf::ParserConfig::URDFPreserveFixedJoint,
         "Get the preserveFixedJoint flag value.")
    .def("__copy__", [](const sdf::ParserConfig &self) {
      return sdf::ParserConfig(self);
    })
    .def("__deepcopy__", [](const sdf::ParserConfig &self, pybind11::dict) {
      return sdf::ParserConfig(self);
    }, "memo"_a);

  pybind11::enum_<sdf::EnforcementPolicy>(
    module, "EnforcementPolicy")
    .value("ERR", sdf::EnforcementPolicy::ERR)
    .value("WARN", sdf::EnforcementPolicy::WARN)
    .value("LOG", sdf::EnforcementPolicy::LOG);

  pybind11::enum_<sdf::ConfigureResolveAutoInertials>(
    module, "ConfigureResolveAutoInertials")
    .value("SKIP_CALCULATION_IN_LOAD", sdf::ConfigureResolveAutoInertials::SKIP_CALCULATION_IN_LOAD)
    .value("SAVE_CALCULATION", sdf::ConfigureResolveAutoInertials::SAVE_CALCULATION);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
