/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "pyProjector.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Projector.hh"
#include "sdf/Plugin.hh"
#include "sdf/Plugin.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineProjector(pybind11::object module)
{
  pybind11::class_<sdf::Projector>(module, "Projector")
    .def(pybind11::init<>())
    // .def(pybind11::init<sdf::Projector>())
    .def("name", &sdf::Projector::Name,
         "Name of the projector.")
    .def("set_name", &sdf::Projector::SetName,
         "Set the name of the projector, which should be unique "
         "within the scope of a Link.")
    .def("near_clip", &sdf::Projector::NearClip,
         "The near clip distance.")
    .def("set_near_clip", &sdf::Projector::SetNearClip,
         "Set the near clip distance.")
    .def("far_clip", &sdf::Projector::FarClip,
         "The far clip distance.")
    .def("set_far_clip", &sdf::Projector::SetFarClip,
         "Set the far clip distance.")
    .def("horizontal_fov", &sdf::Projector::HorizontalFov,
         "Get the horizontal field of view in radians.")
    .def("set_horizontal_fov", &sdf::Projector::SetHorizontalFov,
         "Set the horizontal field of view in radians.")
    .def("visibility_flags", &sdf::Projector::VisibilityFlags,
         "Get the visibility flags of a projector.")
    .def("set_visibility_flags", &sdf::Projector::SetVisibilityFlags,
         "Set the visibility flags of a projector.")
    .def("texture", &sdf::Projector::Texture,
         "Get the texture filename. This will be an empty string if "
         "a texture has not been set.")
    .def("set_texture", &sdf::Projector::SetTexture,
         "Set the texture filename.")
    .def("add_plugin", &sdf::Projector::AddPlugin,
         "Add a plugin to this object.")
    .def("plugins",
         pybind11::overload_cast<>(&sdf::Projector::Plugins),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable vector of plugins attached to this object")
    .def("clear_plugins", &sdf::Projector::ClearPlugins,
         "Remove all plugins.")
    .def("raw_pose", &sdf::Projector::RawPose,
         "Get the pose of the frame object. This is the pose of the "
         "frame as specified in SDF")
    .def("set_raw_pose", &sdf::Projector::SetRawPose,
         "Set the raw pose of the frame object. This is interpreted "
         "relative to the frame named in the //pose/@relative_to attribute.")
    .def("pose_relative_to", &sdf::Projector::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "frame's pose is expressed. An empty value indicates that the frame "
         "is expressed relative to the attached-to link.")
    .def("set_pose_relative_to", &sdf::Projector::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "frame's pose is expressed. An empty value indicates that the frame "
         "is expressed relative to the attached-to link.")
    .def("file_path", &sdf::Projector::FilePath,
         "The path to the file where this element was loaded from.")
    .def("set_file_path", &sdf::Projector::SetFilePath,
         "Set the path to the file where this element was loaded from.")
    .def("semantic_pose", &sdf::Projector::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("__copy__", [](const sdf::Projector &self) {
      return sdf::Projector(self);
    })
    .def("__deepcopy__", [](const sdf::Projector &self, pybind11::dict) {
      return sdf::Projector(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
