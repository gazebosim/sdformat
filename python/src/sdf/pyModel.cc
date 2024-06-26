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

#include "pyModel.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"

#include "pybind11_helpers.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineModel(pybind11::object module)
{
  pybind11::class_<sdf::Model>(module, "Model")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Model>())
    .def("validate_graphs", [](const sdf::Model &self)
         {
           ThrowIfErrors(self.ValidateGraphs());
         },
         "Check that the FrameAttachedToGraph and PoseRelativeToGraph "
         "are valid.")
    .def("resolve_auto_inertials", &sdf::Model::ResolveAutoInertials,
         "Calculate and set the inertials for all the links belonging to the model object")
    .def("name", &sdf::Model::Name,
         "Get the name of model.")
    .def("set_name", &sdf::Model::SetName,
         "Set the name of model.")
    .def("static", &sdf::Model::Static,
         "Check if this model should be static. "
         "A static model is one that is not subject to physical forces (in "
         "other words, it's purely kinematic instead of dynamic).")
    .def("set_static", &sdf::Model::SetStatic,
         "Set this model to be static or not static.")
    .def("self_collide", &sdf::Model::SelfCollide,
         "Check if this model should self-collide.")
    .def("set_self_collide", &sdf::Model::SetSelfCollide,
         "Set this model to self-collide or not self-collide.")
    .def("allow_auto_disable", &sdf::Model::AllowAutoDisable,
         "Check if this model should be allowed to auto-disable. "
         "If auto-disable is allowed, a model that is at rest can choose to "
         "not update its dynamics.")
    .def("set_allow_auto_disable", &sdf::Model::SetAllowAutoDisable,
         "Set this model to allow auto-disabling.")
    .def("enable_wind", &sdf::Model::EnableWind,
         "Check if this model should be subject to wind. "
         "If true, all links in the model should be affected by the wind. This "
         "can be overridden per link.")
    .def("set_enable_wind", &sdf::Model::SetEnableWind,
         "Set whether this model should be subject to wind.")
    .def("link_count", &sdf::Model::LinkCount,
         "Get the number of links that are immediate (not nested) children "
         "of this Model object.")
    .def("link_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Model::LinkByIndex, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) child link based on an index.")
    .def("link_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Model::LinkByName, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) child link based on an index.")
    .def("link_name_exists", &sdf::Model::LinkNameExists,
         "Get whether a link name exists.")
    .def("joint_count", &sdf::Model::JointCount,
         "Get the number of joints that are immediate (not nested) children "
         "of this Model object.")
    .def("joint_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Model::JointByIndex, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) child joint based on an index.")
    .def("joint_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Model::JointByName, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) mutable child joint based on an "
         "index.")
    .def("joint_name_exists", &sdf::Model::JointNameExists,
         "Get whether a joint name exists.")
    .def("frame_count", &sdf::Model::FrameCount,
         "Get the number of explicit frames that are immediate (not nested) "
         "children of this Model object.")
    .def("frame_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Model::FrameByIndex, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) child joint frame on an index.")
    .def("frame_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Model::FrameByName, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) mutable child frame based on an "
         "index.")
    .def("frame_name_exists", &sdf::Model::FrameNameExists,
         "Get whether a frame name exists.")
    .def("model_count", &sdf::Model::ModelCount,
         "Get the number of explicit model that are immediate (not nested) "
         "children of this Model object.")
    .def("model_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Model::ModelByIndex, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) child joint model on an index.")
    .def("model_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Model::ModelByName, pybind11::const_),
         pybind11::return_value_policy::reference_internal,
         "Get an immediate (not nested) mutable child model based on an "
         "index.")
    .def("model_name_exists", &sdf::Model::ModelNameExists,
         "Get whether a model name exists.")
    .def("raw_pose", &sdf::Model::RawPose,
         "Get the pose of the model. This is the pose of the model "
         "as specified in SDF (<model> <pose> ... </pose></model>), and is "
         "typically used to express the position and rotation of a model in a "
         "global coordinate frame.")
    .def("set_raw_pose", &sdf::Model::SetRawPose,
         "Set the pose of the model.")
    .def("canonical_link", &sdf::Model::CanonicalLink,
         pybind11::return_value_policy::reference_internal,
         "Get the model's canonical link")
    .def("canonical_link_name", &sdf::Model::CanonicalLinkName,
         "Get the name of the model's canonical link. An empty value "
         "indicates that the first link in the model or the first link found "
         "in a depth first search of nested models is the canonical link.")
    .def("set_canonical_link_name", &sdf::Model::SetCanonicalLinkName,
         "Set the name of the model's canonical link. An empty value "
         "indicates that the first link in the model or the first link found "
         "in a depth first search of nested models is the canonical link.")
    .def("pose_relative_to", &sdf::Model::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "frame's pose is expressed. An empty value indicates that the frame "
         "is expressed relative to the attached-to link.")
    .def("set_pose_relative_to", &sdf::Model::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "frame's pose is expressed. An empty value indicates that the frame "
         "is expressed relative to the attached-to link.")
    .def("semantic_pose", &sdf::Model::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("placement_frame_name", &sdf::Model::PlacementFrameName,
         "Get the name of the placement frame of the model.")
    .def("set_placement_frame_name", &sdf::Model::SetPlacementFrameName,
         "Set the name of the placement frame of the model.")
    .def("canonical_link_and_relative_name", &sdf::Model::CanonicalLinkAndRelativeName,
         pybind11::return_value_policy::reference_internal,
         "Get the model's canonical link and the nested name of the link "
         "relative to the current model, delimited by \"::\".")
     .def("name_exists_in_frame_attached_to_graph",
          &sdf::Model::NameExistsInFrameAttachedToGraph,
          "Check if a given name exists in the FrameAttachedTo graph at the "
          "scope of the model.")
     .def("add_link", &sdf::Model::AddLink,
          "Add a link to the model.")
     .def("add_joint", &sdf::Model::AddJoint,
          "Add a joint to the model.")
     .def("add_model", &sdf::Model::AddModel,
          "Add a model to the model.")
     .def("add_frame", &sdf::Model::AddFrame,
          "Add a frame to the model.")
     .def("clear_links", &sdf::Model::ClearLinks,
          "Remove all links.")
     .def("clear_joints", &sdf::Model::ClearJoints,
          "Remove all joints.")
     .def("clear_models", &sdf::Model::ClearModels,
          "Remove all models.")
     .def("clear_frames", &sdf::Model::ClearFrames,
          "Remove all frames.")
     .def("uri", &sdf::Model::Uri,
          "Get the URI associated with this model")
     .def("set_uri", &sdf::Model::SetUri,
          "Set the URI associated with this model.")
     .def("plugins",
          pybind11::overload_cast<>(&sdf::Model::Plugins, pybind11::const_),
          "Get the plugins attached to this object.")
     .def("clear_plugins", &sdf::Model::ClearPlugins,
          "Remove all plugins")
     .def("add_plugin", &sdf::Model::AddPlugin,
          "Add a plugin to this object.")
    .def("__copy__", [](const sdf::Model &self) {
      return sdf::Model(self);
    })
    .def("__deepcopy__", [](const sdf::Model &self, pybind11::dict) {
      return sdf::Model(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
