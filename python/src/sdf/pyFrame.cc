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

#include "pyFrame.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Frame.hh"
#include "pyExceptions.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineFrame(pybind11::object module)
{
  pybind11::class_<sdf::Frame>(module, "Frame")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Frame>())
    .def("name", &sdf::Frame::Name,
         "Get the name of the frame.")
    .def("set_name", &sdf::Frame::SetName,
         "Set the name of the frame.")
    .def("attached_to", &sdf::Frame::AttachedTo,
         "Get the name of the coordinate frame to which this "
         "frame is attached.")
    .def("set_attached_to", &sdf::Frame::SetAttachedTo,
         "Set the name of the coordinate frame to which this "
         "frame is attached.")
    .def("raw_pose", &sdf::Frame::RawPose,
         "Get the pose of the frame object. This is the pose of the "
         "frame as specified in SDF")
    .def("set_raw_pose", &sdf::Frame::SetRawPose,
         "Set the raw pose of the frame object. This is interpreted "
         "relative to the frame named in the //pose/@relative_to attribute.")
    .def("pose_relative_to", &sdf::Frame::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "frame's pose is expressed. An empty value indicates that the frame "
         "is expressed relative to the attached-to link.")
    .def("set_pose_relative_to", &sdf::Frame::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "frame's pose is expressed. An empty value indicates that the frame "
         "is expressed relative to the attached-to link.")
    .def("resolve_attached_to_body",
         [](const sdf::Frame &self)
         {
           std::string body;
           auto errors = self.ResolveAttachedToBody(body);
           if (!errors.empty())
           {
             throw SDFErrorsException(errors);
           }
           return std::make_tuple(errors, body);
         },
         "Resolve the attached-to body of this frame from the "
         "FrameAttachedToGraph. Generally, it resolves to the name of a link, "
         "but if it is in the world scope, it can resolve to \"world\".")
    .def("semantic_pose", &sdf::Frame::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("__copy__", [](const sdf::Frame &self) {
      return sdf::Frame(self);
    })
    .def("__deepcopy__", [](const sdf::Frame &self, pybind11::dict) {
      return sdf::Frame(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
