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

#include "pyAtmosphere.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <gz/math/Temperature.hh>

#include "sdf/Atmosphere.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineAtmosphere(pybind11::object module)
{
  pybind11::class_<sdf::Atmosphere> atmosphereModule(module, "Atmosphere");
  atmosphereModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Atmosphere>())
    .def(pybind11::self == pybind11::self)
    .def("type", &sdf::Atmosphere::Type,
         "Get the type of the atmospheric model.")
    .def("set_type", &sdf::Atmosphere::SetType,
         "Set the type of the atmospheric model.")
    .def("temperature", &sdf::Atmosphere::Temperature,
         "Get the temperature at sea level.")
    .def("set_temperature", &sdf::Atmosphere::SetTemperature,
         "Set the temperature at sea level")
    .def("temperature_gradient", &sdf::Atmosphere::TemperatureGradient,
         "Get the temperature gradient with respect to increasing "
         "altitude in units of K/m.")
    .def("set_temperature_gradient", &sdf::Atmosphere::SetTemperatureGradient,
         "Set the temperature gradient with respect to increasing "
         "altitude in units of K/m.")
    .def("pressure", &sdf::Atmosphere::Pressure,
         "Get the pressure at sea level in pascals.")
    .def("set_pressure", &sdf::Atmosphere::SetPressure,
         "Set the pressure at sea level in pascals.")
    .def("__copy__", [](const sdf::Atmosphere &self) {
      return sdf::Atmosphere(self);
    })
    .def("__deepcopy__", [](const sdf::Atmosphere &self, pybind11::dict) {
      return sdf::Atmosphere(self);
    }, "memo"_a);

    pybind11::enum_<sdf::AtmosphereType>(module, "AtmosphereType")
      .value("ADIABATIC", sdf::AtmosphereType::ADIABATIC);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
