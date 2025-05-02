/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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
#include "pyCone.hh"

#include <pybind11/pybind11.h>

#include "sdf/Cone.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineCone(pybind11::object module)
{
  pybind11::class_<sdf::Cone>(module, "Cone")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Cone>())
    .def("radius", &sdf::Cone::Radius,
         "Get the cone's radius in meters.")
    .def("set_radius", &sdf::Cone::SetRadius,
         "Set the cone's radius in meters.")
    .def("length", &sdf::Cone::Length,
         "Get the cone's length in meters.")
    .def("set_length", &sdf::Cone::SetLength,
         "Set the cone's length in meters.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Cone::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Gazebo Math representation of this Cone.")
    .def("axis_aligned_box", &sdf::Cone::AxisAlignedBox,
         "Get the axis-aligned box that contains this Cone.")
    .def("__copy__", [](const sdf::Cone &self) {
      return sdf::Cone(self);
    })
    .def("__deepcopy__", [](const sdf::Cone &self, pybind11::dict) {
      return sdf::Cone(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
