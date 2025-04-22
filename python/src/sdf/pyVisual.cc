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

#include "pyVisual.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"
#include "sdf/Material.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineVisual(pybind11::object module)
{
  pybind11::class_<sdf::Visual>(module, "Visual")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Visual>())
    .def("name", &sdf::Visual::Name,
         "Get the name of the visual. "
         "The name of the visual must be unique within the scope of a Link.")
    .def("set_name", &sdf::Visual::SetName,
         "Set the name of the visual. "
         "The name of the visual must be unique within the scope of a Link.")
    .def("cast_shadows", &sdf::Visual::CastShadows,
         "Get whether the visual casts shadows")
    .def("set_cast_shadows", &sdf::Visual::SetCastShadows,
         "Set whether the visual casts shadows")
    .def("transparency", &sdf::Visual::Transparency,
         "Get the transparency value of the visual")
    .def("set_transparency", &sdf::Visual::SetTransparency,
         "Set the transparency value for the visual")
    .def("geometry", &sdf::Visual::Geom,
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to the visual's geometry.")
    .def("set_geometry", &sdf::Visual::SetGeom,
         "Set the visual's geometry")
    .def("raw_pose", &sdf::Visual::RawPose,
         "Get the pose of the visual object. This is the pose of the "
         "collision as specified in SDF")
    .def("set_raw_pose", &sdf::Visual::SetRawPose,
         "Set the pose of the visual object.")
    .def("pose_relative_to", &sdf::Visual::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("set_pose_relative_to", &sdf::Visual::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("semantic_pose", &sdf::Visual::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("material", &sdf::Visual::Material,
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to the visual's material properties. This can"
         "be a nullptr if material properties have not been set.")
    .def("set_material", &sdf::Visual::SetMaterial,
         "Set the visual's material")
    .def("visibility_flags", &sdf::Visual::VisibilityFlags,
         "Get the visibility flags of a visual")
    .def("set_visibility_flags", &sdf::Visual::SetVisibilityFlags,
         "Set the visibility flags of a visual")
    .def("set_has_laser_retro", &sdf::Visual::SetHasLaserRetro,
         "Set whether the lidar reflective intensity has been specified.")
    .def("has_laser_retro", &sdf::Visual::HasLaserRetro,
         "Get whether the lidar reflective intensity was set was set.")
    .def("laser_retro", &sdf::Visual::LaserRetro,
         "Get the lidar reflective intensity.")
    .def("set_laser_retro", &sdf::Visual::SetLaserRetro,
         "Set the lidar reflective intensity.")
    .def("plugins",
         pybind11::overload_cast<>(&sdf::Visual::Plugins, pybind11::const_),
         "Get the plugins attached to this object.")
    .def("clear_plugins", &sdf::Visual::ClearPlugins,
         "Remove all plugins")
    .def("add_plugin", &sdf::Visual::AddPlugin,
         "Add a plugin to this object.")
    .def("__copy__", [](const sdf::Visual &self) {
      return sdf::Visual(self);
    })
    .def("__deepcopy__", [](const sdf::Visual &self, pybind11::dict) {
      return sdf::Visual(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
