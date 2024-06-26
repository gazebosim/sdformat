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
#include "pySphere.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Sphere.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineSphere(pybind11::object module)
{
  pybind11::class_<sdf::Sphere>(module, "Sphere")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Sphere>())
    .def("calculate_inertial", &sdf::Sphere::CalculateInertial,
         "Calculate and return the Inertial values for the Sphere.")
    .def("radius", &sdf::Sphere::Radius,
         "Get the sphere's radius in meters.")
    .def("set_radius", &sdf::Sphere::SetRadius,
         "Set the sphere's radius in meters.")
    .def(
        "shape",
        pybind11::overload_cast<>(&sdf::Sphere::Shape, pybind11::const_),
        pybind11::return_value_policy::reference,
        "Get a mutable Gazebo Math representation of this Sphere.")
    .def("__copy__", [](const sdf::Sphere &self) {
      return sdf::Sphere(self);
    })
    .def("__deepcopy__", [](const sdf::Sphere &self, pybind11::dict) {
      return sdf::Sphere(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
