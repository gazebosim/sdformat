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

#include "pyJointAxis.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/JointAxis.hh"


using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineJointAxis(pybind11::object module)
{
  pybind11::class_<sdf::JointAxis>(module, "JointAxis")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::JointAxis>())
    .def("xyz", &sdf::JointAxis::Xyz,
         "Get the x,y,z components of the axis unit vector.")
    .def("set_xyz", &sdf::JointAxis::SetXyz,
         "Set the x,y,z components of the axis unit vector.")
    .def("damping", &sdf::JointAxis::Damping,
         "Get the physical velocity dependent viscous damping coefficient "
         "of the joint axis. The default value is zero (0.0).")
    .def("set_damping", &sdf::JointAxis::SetDamping,
         "Set the physical velocity dependent viscous damping coefficient "
         "of the joint axis.")
    .def("friction", &sdf::JointAxis::Friction,
         "Get the physical static friction value of the joint. The "
         "default value is zero (0.0).")
    .def("set_friction", &sdf::JointAxis::SetFriction,
         "Set the physical static friction value of the joint.")
    .def("spring_reference", &sdf::JointAxis::SpringReference,
         "Get the spring reference position for this joint axis. The "
         "default value is zero (0.0).")
    .def("set_spring_reference", &sdf::JointAxis::SetSpringReference,
         "Set the spring reference position for this joint axis.")
    .def("spring_stiffness", &sdf::JointAxis::SpringStiffness,
         "Get the spring stiffness for this joint axis. The default "
         "value is zero (0.0).")
    .def("set_spring_stiffness", &sdf::JointAxis::SetSpringStiffness,
         "Set the spring stiffness for this joint axis.")
    .def("lower", &sdf::JointAxis::Lower,
         " Get the lower joint axis limit (radians for revolute joints, "
         "meters for prismatic joints). Not valid if the joint that uses this "
         "axis is continuous. The default value is -1e16.")
    .def("set_lower", &sdf::JointAxis::SetLower,
         "Set the lower joint axis limit (radians for revolute joints, "
         "meters for prismatic joints). Not valid if the joint that uses this "
         "axis is continuous.")
    .def("upper", &sdf::JointAxis::Upper,
         "Get the upper joint axis limit (radians for revolute joints, "
         "meters for prismatic joints). Not valid if joint that uses this "
         "axis is continuous. The default value is 1e16.")
    .def("set_upper", &sdf::JointAxis::SetUpper,
         "Set the upper joint axis limit (radians for revolute joints, "
         "meters for prismatic joints). Not valid if joint that uses this "
         "axis is continuous.")
    .def("effort", &sdf::JointAxis::Effort,
         "Get the value for enforcing the maximum absolute joint effort "
         "that can be applied.")
    .def("set_effort", &sdf::JointAxis::SetEffort,
         "Set the value for enforcing the maximum absolute joint effort "
         "that can be applied.")
    .def("max_velocity", &sdf::JointAxis::MaxVelocity,
         "Get the value for enforcing the maximum absolute joint velocity. "
         "The default value is infinity.")
    .def("set_max_velocity", &sdf::JointAxis::SetMaxVelocity,
         "Set the value for enforcing the maximum absolute joint velocity.")
    .def("stiffness", &sdf::JointAxis::Stiffness,
         "Get the joint stop stiffness. The default value is 1e8.")
    .def("set_stiffness", &sdf::JointAxis::SetStiffness,
         "Set the joint stop stiffness.")
    .def("dissipation", &sdf::JointAxis::Dissipation,
         "Get the joint stop dissipation. The default value is 1.0.")
    .def("set_dissipation", &sdf::JointAxis::SetDissipation,
         "Set the joint stop dissipation.")
    .def("xyz_expressed_in", &sdf::JointAxis::XyzExpressedIn,
         "Get the name of the coordinate frame in which this joint axis's "
         "unit vector is expressed. An empty value implies the parent (joint) "
         "frame.")
    .def("set_xyz_expressed_in", &sdf::JointAxis::SetXyzExpressedIn,
         "Set the name of the coordinate frame in which this joint axis's "
         "unit vector is expressed. An empty value implies the parent (joint) "
         "frame.")
    .def("resolve_xyz", &sdf::JointAxis::ResolveXyz,
         pybind11::arg("_xyz"),
         pybind11::arg("_resolveTo") = "",
         "Express xyz unit vector of this axis in the coordinates of "
         "another named frame.")
    .def("__copy__", [](const sdf::JointAxis &self) {
      return sdf::JointAxis(self);
    })
    .def("__deepcopy__", [](const sdf::JointAxis &self, pybind11::dict) {
      return sdf::JointAxis(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
