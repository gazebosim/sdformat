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

#include "pyAirSpeed.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/AirSpeed.hh"


using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineAirSpeed(pybind11::object module)
{
  pybind11::class_<sdf::AirSpeed> geometryModule(module, "AirSpeed");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::AirSpeed>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("pressure_noise", &sdf::AirSpeed::PressureNoise,
         "Get the noise values.")
    .def("set_pressure_noise",
         &sdf::AirSpeed::SetPressureNoise,
         "Set the noise values related to the pressure data.")
    .def("__copy__", [](const sdf::AirSpeed &self) {
      return sdf::AirSpeed(self);
    })
    .def("__deepcopy__", [](const sdf::AirSpeed &self, pybind11::dict) {
      return sdf::AirSpeed(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
