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

#include "pyAirFlow.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/AirFlow.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineAirFlow(pybind11::object module)
{
  pybind11::class_<sdf::AirFlow> geometryModule(module, "AirFlow");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::AirFlow>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("speed_noise", &sdf::AirFlow::SpeedNoise,
         "Get the speed noise values.")
    .def("set_speed_noise",
         &sdf::AirFlow::SetSpeedNoise,
         "Set the noise values related to the speed data.")
    .def("direction_noise", &sdf::AirFlow::DirectionNoise,
         "Get the direction noise values.")
    .def("set_direction_noise",
         &sdf::AirFlow::SetDirectionNoise,
         "Set the noise values related to the direction data.")
    .def("__copy__", [](const sdf::AirFlow &self) {
      return sdf::AirFlow(self);
    })
    .def("__deepcopy__", [](const sdf::AirFlow &self, pybind11::dict) {
      return sdf::AirFlow(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
