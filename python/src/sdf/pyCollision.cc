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

#include "pyCollision.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pybind11_helpers.hh"
#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Surface.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineCollision(pybind11::object module)
{
  pybind11::class_<sdf::Collision> geometryModule(module, "Collision");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Collision>())
    .def_static("density_default", &sdf::Collision::DensityDefault,
         "Get the default density of a collision if its density is not "
         "specified.")
    .def("set_density", &sdf::Collision::SetDensity, "Set the density of the collision.")
    .def("density", &sdf::Collision::Density, "Get the density of the collision.")
    .def("auto_inertia_params", &sdf::Collision::AutoInertiaParams,
         "Get the <auto_inertia_params> element. This element can be used "
         "as a parent element to hold user-defined params for the custom "
         "moment of inertia calculator.")
    .def("set_auto_inertia_params", &sdf::Collision::SetAutoInertiaParams,
         "Set the <auto_inertia_params> element.")
    .def("calculate_inertial",
         pybind11::overload_cast<sdf::Errors &, gz::math::Inertiald &, const ParserConfig &>(
          &sdf::Collision::CalculateInertial),
         "Calculate and return the MassMatrix for the collision.")
    .def("calculate_inertial",
         ErrorWrappedCast<gz::math::Inertiald &, const sdf::ParserConfig &,
                          const std::optional<double> &, sdf::ElementPtr,
                          bool>(&sdf::Collision::CalculateInertial),
         "inertial"_a, "config"_a, "density"_a, "auto_inertia_params"_a,
         "warn_if_density_is_unset"_a = true,
         "Calculate the inertial values for the collision. The density and "
         "<auto_inertia_params> element are used if they have not been set "
         "in this collision.")
    .def("name", &sdf::Collision::Name,
         "Get the name of the collision. "
         "The name of the collision must be unique within the scope of a Link.")
    .def("set_name", &sdf::Collision::SetName,
         "Set the name of the collision. "
         "The name of the collision must be unique within the scope of a Link.")
    .def("geometry", &sdf::Collision::Geom,
         pybind11::return_value_policy::reference,
         "Get a pointer to the collisions's geometry.")
    .def("set_geometry", &sdf::Collision::SetGeom,
         "Set the collision's geometry")
    .def("surface", &sdf::Collision::Surface,
         pybind11::return_value_policy::reference,
         "Get a pointer to the collisions's surface parameters.")
    .def("set_surface", &sdf::Collision::SetSurface,
         "Set the collision's surface parameters")
    .def("raw_pose", &sdf::Collision::RawPose,
         "Get the pose of the collision object. This is the pose of the "
         "collision as specified in SDF")
    .def("set_raw_pose", &sdf::Collision::SetRawPose,
         "Set the pose of the collision object.")
    .def("pose_relative_to", &sdf::Collision::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("set_pose_relative_to", &sdf::Collision::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("semantic_pose", &sdf::Collision::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("__copy__", [](const sdf::Collision &self) {
      return sdf::Collision(self);
    })
    .def("__deepcopy__", [](const sdf::Collision &self, pybind11::dict) {
      return sdf::Collision(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
