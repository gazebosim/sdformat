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

#include "pyMagnetometer.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Magnetometer.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineMagnetometer(pybind11::object module)
{
  pybind11::class_<sdf::Magnetometer> geometryModule(module, "Magnetometer");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Magnetometer>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("x_noise", &sdf::Magnetometer::XNoise,
         "Get the noise values related to the body-frame x axis.")
    .def("set_x_noise", &sdf::Magnetometer::SetXNoise,
         "Set the noise values related to the body-frame x axis.")
    .def("y_noise", &sdf::Magnetometer::YNoise,
         "Get the noise values related to the body-frame y axis.")
    .def("set_y_noise", &sdf::Magnetometer::SetYNoise,
        "Set the noise values related to the body-frame y axis.")
    .def("z_noise", &sdf::Magnetometer::ZNoise,
         "Get the noise values related to the body-frame z axis.")
    .def("set_z_noise", &sdf::Magnetometer::SetZNoise,
         "Set the noise values related to the body-frame z axis.")
    .def("__copy__", [](const sdf::Magnetometer &self) {
      return sdf::Magnetometer(self);
    })
    .def("__deepcopy__", [](const sdf::Magnetometer &self, pybind11::dict) {
      return sdf::Magnetometer(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
