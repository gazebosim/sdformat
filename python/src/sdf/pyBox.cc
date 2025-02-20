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
#include "pyBox.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Box.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineBox(pybind11::object module)
{
  pybind11::class_<sdf::Box>(module, "Box")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Box>())
    .def("size", &sdf::Box::Size,
         "Get the box size in meters.")
    .def("set_size", &sdf::Box::SetSize,
         "Set the box size in meters.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Box::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Gazebo Math representation of this Box.")
    .def("calculate_inertial", &sdf::Box::CalculateInertial,
         "Calculate and return the Inertial values for the Box.")
    .def("axis_aligned_box", &sdf::Box::AxisAlignedBox,
         "Get the axis-aligned box that contains this box.")
    .def("__copy__", [](const sdf::Box &self) {
      return sdf::Box(self);
    })
    .def("__deepcopy__", [](const sdf::Box &self, pybind11::dict) {
      return sdf::Box(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
