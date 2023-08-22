/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "pyPrintConfig.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "pybind11_helpers.hh"
#include "sdf/PrintConfig.hh"
#include "sdf/config.hh"

using namespace pybind11::literals;
namespace py = pybind11;
namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{

void definePrintConfig(py::object module)
{
  py::class_<sdf::PrintConfig>(module, "PrintConfig")
      .def(py::init<>())
      .def(py::init<PrintConfig>())
      .def(py::self == py::self)
      .def("set_rotation_in_degrees", &PrintConfig::SetRotationInDegrees,
           "Sets the option for printing pose rotations in degrees if true, "
           "otherwise they will be printed as radians by default.")
      .def("rotation_in_degrees", &PrintConfig::RotationInDegrees,
           "Returns whether or not pose rotations should be printed in "
           "degrees.")
      .def("set_rotation_snap_to_degrees",
           ErrorWrappedCast<unsigned int, double>(
               &PrintConfig::SetRotationSnapToDegrees),
           "Sets the option for printing pose rotation in degrees as well as "
           "snapping the rotation to the desired interval, with the provided "
           "tolerance.")
      .def("rotation_snap_to_degrees", &PrintConfig::RotationSnapToDegrees,
           "Returns the current degree value that pose rotations will snap to "
           "when printed.")
      .def("rotation_snap_tolerance", &PrintConfig::RotationSnapTolerance,
           "Returns the tolerance for snapping degree values when printed.")
      .def("set_preserve_includes", &PrintConfig::SetPreserveIncludes,
           "Set print config to preserve <include> tags.")
      .def("preserve_includes", &PrintConfig::PreserveIncludes,
           "Check if <include> tags are to be preserved or expanded.")
      .def("set_out_precision", &PrintConfig::SetOutPrecision,
           "Set precision of output stream for float / double types. By "
           "default, the output stream uses maximum precision.")
      .def("out_precision", &PrintConfig::OutPrecision,
           "Retrieve the output stream's set precision value.")
      .def("__copy__",
           [](const PrintConfig &self) { return PrintConfig(self); })
      .def(
          "__deepcopy__",
          [](const PrintConfig &self, py::dict) { return PrintConfig(self); },
          "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
