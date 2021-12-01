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

#ifndef SDF_INTERFACE_MODEL_POSE_GRAPH
#define SDF_INTERFACE_MODEL_POSE_GRAPH

#include <string>
#include <memory>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/Types.hh"

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
// Forward declarations.
class InterfaceModel;
struct PoseRelativeToGraph;
template <typename T>
class ScopedGraph;

/// \brief Class used in reposture callbacks of custom parsers to resolve poses.
class SDFORMAT_VISIBLE InterfaceModelPoseGraph
{
  /// \brief Resolve pose relative to world
  /// \param[out] _pose Resolved pose
  /// \return Errors
  public: sdf::Errors ResolveNestedModelFramePoseInWorldFrame(
              ignition::math::Pose3d &_pose) const;

  /// \brief Resolve the pose a frame within the model's scope.
  /// \param[in] relative_to Can be "world", or any frame within the nested
  ///   model's frame graph. (It cannot reach outside of this model).
  public: sdf::Errors ResolveNestedFramePose(ignition::math::Pose3d &_pose,
              const std::string &_frameName,
              const std::string &_relativeTo = "world") const;

  /// \brief Private constructor
  /// \param[in] _name Interface model associated with this object
  /// \param[in] _graph Pose relative-to graph at the scope of the interface
  /// model associated with this object.
  private: InterfaceModelPoseGraph(const std::string &_name,
               const sdf::ScopedGraph<sdf::PoseRelativeToGraph> &_graph);

  friend class InterfaceModel;
  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
