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
#include "pyCylinder.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Cylinder.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineCylinder(pybind11::object module)
{
  pybind11::class_<sdf::Cylinder>(module, "Cylinder")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Cylinder>())
    .def("calculate_inertial", &sdf::Cylinder::CalculateInertial,
         "Calculate and return the Inertial values for the Cylinder.")
    .def("radius", &sdf::Cylinder::Radius,
         "Get the cylinder's radius in meters.")
    .def("set_radius", &sdf::Cylinder::SetRadius,
         "Set the cylinder's radius in meters.")
    .def("length", &sdf::Cylinder::Length,
         "Get the cylinder's length in meters.")
    .def("set_length", &sdf::Cylinder::SetLength,
         "Set the cylinder's length in meters.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Cylinder::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Gazebo Math representation of this Cylinder.")
    .def("axis_aligned_box", &sdf::Cylinder::AxisAlignedBox,
         "Get the axis-aligned box that contains this Cylinder.")
    .def("__copy__", [](const sdf::Cylinder &self) {
      return sdf::Cylinder(self);
    })
    .def("__deepcopy__", [](const sdf::Cylinder &self, pybind11::dict) {
      return sdf::Cylinder(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
