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
#include "pyPlane.hh"

#include <pybind11/pybind11.h>

#include "sdf/Plane.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void definePlane(pybind11::object module)
{
  pybind11::class_<sdf::Plane>(module, "Plane")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Plane>())
    .def("size", &sdf::Plane::Size,
         "Get the plane size in meters.")
    .def("set_size", &sdf::Plane::SetSize,
         "Set the plane size in meters.")
    .def("normal", &sdf::Plane::Normal,
          "Get the plane normal vector. When a Plane is used as a geometry "
          "for a Visual or Collision object, then the normal is specified in "
          "the Visual or Collision frame, respectively.")
    .def("set_normal", &sdf::Plane::SetNormal,
          "Set the plane normal vector. The normal vector will be "
          "normalized. See ignition::math::Vector3d Normal() for more "
          "information about the normal vector, such as the frame in which it "
          "is specified.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Plane::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Ignition Math representation of this Plane.")
    .def("__copy__", [](const sdf::Plane &self) {
      return sdf::Plane(self);
    })
    .def("__deepcopy__", [](const sdf::Plane &self, pybind11::dict) {
      return sdf::Plane(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
