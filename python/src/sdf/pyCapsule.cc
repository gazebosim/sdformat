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
#include "pyCapsule.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Capsule.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineCapsule(pybind11::object module)
{
  pybind11::class_<sdf::Capsule>(module, "Capsule")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Capsule>())
    .def("radius", &sdf::Capsule::Radius,
         "Get the capsule's radius in meters.")
    .def("set_radius", &sdf::Capsule::SetRadius,
         "Set the capsule's radius in meters.")
    .def("length", &sdf::Capsule::Length,
         "Get the capsule's length in meters.")
    .def("set_length", &sdf::Capsule::SetLength,
         "Set the capsule's length in meters.")
    .def("calculate_inertial", &sdf::Capsule::CalculateInertial,
         "Calculate and return the Inertial values for the Capsule.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Capsule::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Gazebo Math representation of this Capsule.")
    .def("__copy__", [](const sdf::Capsule &self) {
      return sdf::Capsule(self);
    })
    .def("__deepcopy__", [](const sdf::Capsule &self, pybind11::dict) {
      return sdf::Capsule(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
