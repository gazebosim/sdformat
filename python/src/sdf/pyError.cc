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
#include "pyError.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <string>

#include "sdf/Error.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineError(pybind11::object module)
{
  pybind11::class_<sdf::Error> errorModule(module, "Error");
  auto toString = [](const sdf::Error &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  errorModule
    .def(pybind11::init<>())
    .def(pybind11::init<const sdf::ErrorCode, const std::string &>())
    .def(pybind11::init<const sdf::ErrorCode,
                        const std::string &,
                        const std::string &>())
    .def(pybind11::init<const sdf::ErrorCode,
                        const std::string &,
                        const std::string &,
                        int>())
    .def("code", &sdf::Error::Code,
         "Get the error code.")
    .def("message", &sdf::Error::Message,
         "Get the error message, which is a description of the error.")
    .def(
        "file_path", &sdf::Error::FilePath,
        "Get the file path associated with this error.")
    .def(
        "set_file_path", &sdf::Error::SetFilePath,
        "Sets the file path that is associated with this error.")
    .def(
        "line_number", &sdf::Error::LineNumber,
        "Get the line number associated with this error.")
    .def(
        "set_line_number", &sdf::Error::SetLineNumber,
        "Sets the line number that is associated with this error.")
    .def(
        "xml_path", &sdf::Error::XmlPath,
        "Get the XPath-like trace that is associated with this error.")
    .def(
        "set_xml_path", &sdf::Error::SetXmlPath,
        "Sets the XML path that is associated with this error.")
    .def(
        "is_valid",
        [](const sdf::Error &self)
        {
          return self == true;
        },
        "True if this Error's Code() != NONE. In otherwords, this is "
        "true when there is an error, or false otherwise.")
    .def("__str__", toString)
    .def("__repr__", toString)
    .def("__copy__", [](const sdf::Error &self) {
      return sdf::Error(self);
    })
    .def("__deepcopy__", [](const sdf::Error &self, pybind11::dict) {
      return sdf::Error(self);
    }, "memo"_a);

  pybind11::enum_<sdf::ErrorCode>(module, "ErrorCode")
    .value("NONE", sdf::ErrorCode::NONE)
    .value("FILE_READ", sdf::ErrorCode::FILE_READ)
    .value("DUPLICATE_NAME", sdf::ErrorCode::DUPLICATE_NAME)
    .value("RESERVED_NAME", sdf::ErrorCode::RESERVED_NAME)
    .value("ATTRIBUTE_MISSING", sdf::ErrorCode::ATTRIBUTE_MISSING)
    .value("ATTRIBUTE_INVALID", sdf::ErrorCode::ATTRIBUTE_INVALID)
    .value("ATTRIBUTE_DEPRECATED", sdf::ErrorCode::ATTRIBUTE_DEPRECATED)
    .value("ATTRIBUTE_INCORRECT_TYPE",
           sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE)
    .value("ELEMENT_MISSING", sdf::ErrorCode::ELEMENT_MISSING)
    .value("ELEMENT_INVALID", sdf::ErrorCode::ELEMENT_INVALID)
    .value("ELEMENT_DEPRECATED", sdf::ErrorCode::ELEMENT_DEPRECATED)
    .value("ELEMENT_INCORRECT_TYPE",
           sdf::ErrorCode::ELEMENT_INCORRECT_TYPE)
    .value("ELEMENT_ERROR", sdf::ErrorCode::ELEMENT_ERROR)
    .value("URI_INVALID", sdf::ErrorCode::URI_INVALID)
    .value("URI_LOOKUP", sdf::ErrorCode::URI_LOOKUP)
    .value("DIRECTORY_NONEXISTANT",
           sdf::ErrorCode::DIRECTORY_NONEXISTANT)
    .value("MODEL_CANONICAL_LINK_INVALID",
           sdf::ErrorCode::MODEL_CANONICAL_LINK_INVALID)
    .value("MODEL_WITHOUT_LINK", sdf::ErrorCode::MODEL_WITHOUT_LINK)
    .value("NESTED_MODELS_UNSUPPORTED",
           sdf::ErrorCode::NESTED_MODELS_UNSUPPORTED)
    .value("LINK_INERTIA_INVALID", sdf::ErrorCode::LINK_INERTIA_INVALID)
    .value("JOINT_CHILD_LINK_INVALID",
           sdf::ErrorCode::JOINT_CHILD_LINK_INVALID)
    .value("JOINT_PARENT_LINK_INVALID",
           sdf::ErrorCode::JOINT_PARENT_LINK_INVALID)
    .value("JOINT_PARENT_SAME_AS_CHILD",
           sdf::ErrorCode::JOINT_PARENT_SAME_AS_CHILD)
    .value("FRAME_ATTACHED_TO_INVALID",
           sdf::ErrorCode::FRAME_ATTACHED_TO_INVALID)
    .value("FRAME_ATTACHED_TO_CYCLE",
           sdf::ErrorCode::FRAME_ATTACHED_TO_CYCLE)
    .value("FRAME_ATTACHED_TO_GRAPH_ERROR",
           sdf::ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR)
    .value("POSE_RELATIVE_TO_INVALID",
           sdf::ErrorCode::POSE_RELATIVE_TO_INVALID)
    .value("POSE_RELATIVE_TO_CYCLE",
           sdf::ErrorCode::POSE_RELATIVE_TO_CYCLE)
    .value("POSE_RELATIVE_TO_GRAPH_ERROR",
           sdf::ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR)
    .value("ROTATION_SNAP_CONFIG_ERROR",
           sdf::ErrorCode::ROTATION_SNAP_CONFIG_ERROR)
    .value("STRING_READ", sdf::ErrorCode::STRING_READ)
    .value("MODEL_PLACEMENT_FRAME_INVALID",
           sdf::ErrorCode::MODEL_PLACEMENT_FRAME_INVALID)
    .value("VERSION_DEPRECATED", sdf::ErrorCode::VERSION_DEPRECATED)
    .value("MERGE_INCLUDE_UNSUPPORTED",
           sdf::ErrorCode::MERGE_INCLUDE_UNSUPPORTED)
    .value("PARAMETER_ERROR", sdf::ErrorCode::PARAMETER_ERROR)
    .value("UNKNOWN_PARAMETER_TYPE", sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE)
    .value("FATAL_ERROR", sdf::ErrorCode::FATAL_ERROR)
    .value("WARNING", sdf::ErrorCode::WARNING)
    .value("JOINT_AXIS_EXPRESSED_IN_INVALID", sdf::ErrorCode::JOINT_AXIS_EXPRESSED_IN_INVALID)
    .value("CONVERSION_ERROR", sdf::ErrorCode::CONVERSION_ERROR)
    .value("PARSING_ERROR", sdf::ErrorCode::PARSING_ERROR)
    .value("JOINT_AXIS_MIMIC_INVALID", sdf::ErrorCode::JOINT_AXIS_MIMIC_INVALID);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
