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

#include "pyAirPressure.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/AirPressure.hh"


using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineAirPressure(pybind11::object module)
{
  pybind11::class_<sdf::AirPressure> geometryModule(module, "AirPressure");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::AirPressure>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("reference_altitude", &sdf::AirPressure::ReferenceAltitude,
         "Get the reference altitude of the sensor in meters. This value "
         "can be used by a sensor implementation to augment the altitude of "
         "the sensor. For example, if you are using simulation instead of  "
         "creating a 1000 m mountain model on which to place your sensor, you "
         "could instead set this value to 1000 and place your model on a "
         " ground plane with a Z height of zero.")
    .def("set_reference_altitude",
         &sdf::AirPressure::SetReferenceAltitude,
         "Set the reference altitude of the sensor in meters.")
    .def("pressure_noise", &sdf::AirPressure::PressureNoise,
         "Get the noise values.")
    .def("set_pressure_noise",
         &sdf::AirPressure::SetPressureNoise,
         "Set the noise values related to the pressure data.")
    .def("__copy__", [](const sdf::AirPressure &self) {
      return sdf::AirPressure(self);
    })
    .def("__deepcopy__", [](const sdf::AirPressure &self, pybind11::dict) {
      return sdf::AirPressure(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
