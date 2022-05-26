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

#include "pyNavSat.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/NavSat.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineNavSat(pybind11::object module)
{
  pybind11::class_<sdf::NavSat>(module, "NavSat")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::NavSat>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("set_horizontal_position_noise", &sdf::NavSat::SetHorizontalPositionNoise,
         "Set the noise values for the horizontal position sensor")
    .def("horizontal_position_noise", &sdf::NavSat::HorizontalPositionNoise,
         "Get noise value for horizontal position sensor")
    .def("set_vertical_position_noise", &sdf::NavSat::SetVerticalPositionNoise,
         "Set the noise values for the vertical position sensor")
    .def("vertical_position_noise", &sdf::NavSat::VerticalPositionNoise,
         "Get noise value for vertical position sensor")
    .def("set_horizontal_velocity_noise", &sdf::NavSat::SetHorizontalVelocityNoise,
         "Set the noise values for the horizontal velocity sensor")
    .def("horizontal_velocity_noise", &sdf::NavSat::HorizontalVelocityNoise,
         "Get noise value for horizontal velocity sensor")
    .def("set_vertical_velocity_noise", &sdf::NavSat::SetVerticalVelocityNoise,
         "Set the noise values for the vertical velocity sensor")
    .def("vertical_velocity_noise", &sdf::NavSat::VerticalVelocityNoise,
         "Get noise value for vertical velocity sensor")
    .def("__copy__", [](const sdf::NavSat &self) {
      return sdf::NavSat(self);
    })
    .def("__deepcopy__", [](const sdf::NavSat &self, pybind11::dict) {
      return sdf::NavSat(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
