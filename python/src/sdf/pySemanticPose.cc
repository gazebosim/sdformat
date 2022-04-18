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
#include "pySemanticPose.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ignition/math/Pose3.hh>

#include "sdf/SemanticPose.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineSemanticPose(pybind11::object module)
{
  pybind11::class_<sdf::SemanticPose>(module, "SemanticPose")
    .def(pybind11::init<sdf::SemanticPose>())
    .def("raw_pose", &sdf::SemanticPose::RawPose,
         "Get the raw Pose3 transform.")
    .def("relative_to", &sdf::SemanticPose::RelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the default parent object.")
    .def("resolve", &sdf::SemanticPose::Resolve,
         pybind11::arg("_pose"),
         pybind11::arg("_resolveTo") = "",
         "Resolve pose of this object with respect to another named frame. "
         "If there are any errors resolving the pose, the output will not be "
         "modified.")
    .def("__copy__", [](const sdf::SemanticPose &self) {
      return sdf::SemanticPose(self);
    })
    .def("__deepcopy__", [](const sdf::SemanticPose &self, pybind11::dict) {
      return sdf::SemanticPose(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
