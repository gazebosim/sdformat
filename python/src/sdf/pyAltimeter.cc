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

#include "pyAltimeter.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Altimeter.hh"


using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineAltimeter(pybind11::object module)
{
  pybind11::class_<sdf::Altimeter> geometryModule(module, "Altimeter");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Altimeter>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("vertical_position_noise", &sdf::Altimeter::VerticalPositionNoise,
         "Get the noise values related to the vertical position.")
    .def("set_vertical_position_noise",
         &sdf::Altimeter::SetVerticalPositionNoise,
         "Set the noise values related to the vertical position.")
    .def("vertical_velocity_noise", &sdf::Altimeter::VerticalVelocityNoise,
         "Get the noise values related to the vertical velocity.")
    .def("set_vertical_velocity_noise",
         &sdf::Altimeter::SetVerticalVelocityNoise,
         "Set the noise values related to the vertical velocity.")
    .def("__copy__", [](const sdf::Altimeter &self) {
      return sdf::Altimeter(self);
    })
    .def("__deepcopy__", [](const sdf::Altimeter &self, pybind11::dict) {
      return sdf::Altimeter(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
