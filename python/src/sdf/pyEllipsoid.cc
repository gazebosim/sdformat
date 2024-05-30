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
#include "pyEllipsoid.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Ellipsoid.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineEllipsoid(pybind11::object module)
{
  pybind11::class_<sdf::Ellipsoid>(module, "Ellipsoid")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Ellipsoid>())
    .def("calculate_inertial", &sdf::Ellipsoid::CalculateInertial,
         "Calculate and return the Inertial values for the Ellipsoid.")
    .def("radii", &sdf::Ellipsoid::Radii,
         "Get the ellipsoid's radii in meters.")
    .def("set_radii", &sdf::Ellipsoid::SetRadii,
         "Set the ellipsoid's x, y, and z radii in meters.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Ellipsoid::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Gazebo Math representation of this Ellipsoid.")
    .def("__copy__", [](const sdf::Ellipsoid &self) {
      return sdf::Ellipsoid(self);
    })
    .def("__deepcopy__", [](const sdf::Ellipsoid &self, pybind11::dict) {
      return sdf::Ellipsoid(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
