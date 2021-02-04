/*
 * Copyright 2021 Open Source Robotics Foundation
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
 *
 */

#include "sdf/InterfaceModelPoseGraph.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class InterfaceModelPoseGraph::Implementation
{
};

InterfaceModelPoseGraph::InterfaceModelPoseGraph(
    const std::string &_name, const std::string &_relativeTo,
    const sdf::ScopedGraph<sdf::PoseRelativeToGraph> &_graph)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}
sdf::Errors InterfaceModelPoseGraph::ResolveNestedModelFramePoseInWorldFrame(
    ignition::math::Pose3d &_pose) const
{
  sdf::Errors errors;
  _pose.SetX(1.0);
  _pose.SetY(2.0);
  return errors;
}
}
}
