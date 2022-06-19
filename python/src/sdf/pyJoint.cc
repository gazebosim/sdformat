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

#include "pyJoint.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/utils/SuppressWarning.hh>

#include "sdf/Joint.hh"
#include "sdf/Sensor.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineJoint(pybind11::object module)
{
  pybind11::class_<sdf::Joint> jointModule(module, "Joint");
  jointModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Joint>())
    .def("name", &sdf::Joint::Name,
         "Get the name of joint.")
    .def("set_name", &sdf::Joint::SetName,
         "Set the name of the joint.")
    .def("type", &sdf::Joint::Type,
         "Get the joint type")
    .def("set_type", &sdf::Joint::SetType,
         "Set the joint type.")
    .def("parent_name", &sdf::Joint::ParentName,
         "Get the name of this joint's parent frame.")
    .def("set_parent_name", &sdf::Joint::SetParentName,
         "Set the name of the parent frame.")
    .def("child_name", &sdf::Joint::ChildName,
         "Get the name of this joint's child frame.")
    .def("set_child_name", &sdf::Joint::SetChildName,
         "Set the name of the child frame.")
    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    .def("parent_link_name", &sdf::Joint::ParentLinkName,
         "Get the name of this joint's parent link.")
    .def("set_parent_link_name", &sdf::Joint::SetParentLinkName,
         "Set the name of the parent link.")
    .def("child_link_name", &sdf::Joint::ChildLinkName,
         "Get the name of this joint's child link.")
    .def("set_child_link_name", &sdf::Joint::SetChildLinkName,
         "Set the name of the child link")
    GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
    .def("resolve_child_link",
         [](const sdf::Joint &self)
         {
           std::string link;
           auto errors = self.ResolveChildLink(link);
           return std::make_tuple(errors, link);
         },
         "Resolve the name of the child link from the "
         "FrameAttachedToGraph.")
    .def("resolve_parent_link",
         [](const sdf::Joint &self)
         {
           std::string link;
           auto errors = self.ResolveParentLink(link);
           return std::make_tuple(errors, link);
         },
         "Resolve the name of the parent link from the "
         "FrameAttachedToGraph. It will return the name of a link or "
         "\"world\".")
    .def("axis", &sdf::Joint::Axis,
         pybind11::return_value_policy::reference,
         "Get a joint axis.")
    .def("set_axis", &sdf::Joint::SetAxis,
         "Set a joint axis.")
    .def("raw_pose", &sdf::Joint::RawPose,
         "Get the pose of the joint. This is the pose of the joint "
         "as specified in SDF (<joint> <pose> ... </pose></joint>). "
         "Transformations have not been applied to the return value.")
    .def("set_raw_pose", &sdf::Joint::SetRawPose,
         "Set the pose of the joint.")
    .def("pose_relative_to", &sdf::Joint::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the child link frame.")
    .def("set_pose_relative_to", &sdf::Joint::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the child link frame.")
    .def("thread_pitch", &sdf::Joint::ThreadPitch,
         "Get the thread pitch (only valid for screw joints)")
    .def("set_thread_pitch", &sdf::Joint::SetThreadPitch,
         "Set the thread pitch (only valid for screw joints)")
    .def("semantic_pose", &sdf::Joint::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("sensor_count", &sdf::Joint::SensorCount,
         "Get the number of sensors.")
    .def("sensor_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Joint::SensorByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a sensor based on an index.")
    .def("sensor_name_exists", &sdf::Joint::SensorNameExists,
         "Get whether a sensor name exists.")
    .def("sensor_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Joint::SensorByName),
         pybind11::return_value_policy::reference_internal,
         "Get a sensor based on a name.")
    .def("add_sensor",
         &sdf::Joint::AddSensor,
         "Add a sensor to the link.")
    .def("clear_sensors",
         &sdf::Joint::ClearSensors,
         "Remove all sensors")
    .def("__copy__", [](const sdf::Joint &self) {
      return sdf::Joint(self);
    })
    .def("__deepcopy__", [](const sdf::Joint &self, pybind11::dict) {
      return sdf::Joint(self);
    }, "memo"_a);

    pybind11::enum_<sdf::JointType>(module, "JointType")
      .value("INVALID", sdf::JointType::INVALID)
      .value("BALL", sdf::JointType::BALL)
      .value("CONTINUOUS", sdf::JointType::CONTINUOUS)
      .value("FIXED", sdf::JointType::FIXED)
      .value("GEARBOX", sdf::JointType::GEARBOX)
      .value("PRISMATIC", sdf::JointType::PRISMATIC)
      .value("REVOLUTE", sdf::JointType::REVOLUTE)
      .value("REVOLUTE2", sdf::JointType::REVOLUTE2)
      .value("SCREW", sdf::JointType::SCREW)
      .value("UNIVERSAL", sdf::JointType::UNIVERSAL);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
