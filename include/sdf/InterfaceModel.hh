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

#ifndef SDF_INTERFACE_MODEL_HH_
#define SDF_INTERFACE_MODEL_HH_

#include <functional>
#include <string>
#include <memory>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/InterfaceFrame.hh"
#include "sdf/InterfaceJoint.hh"
#include "sdf/InterfaceLink.hh"
#include "sdf/InterfaceModelPoseGraph.hh"
#include "sdf/Types.hh"

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class InterfaceModel;
class Model;
struct PoseRelativeToGraph;
template <typename T>
class ScopedGraph;
class World;

using InterfaceModelPtr = std::shared_ptr<InterfaceModel>;
using InterfaceModelConstPtr = std::shared_ptr<const InterfaceModel>;

/// \brief Function signature for the reposture callback function
using RepostureFunction =
    std::function<void(const sdf::InterfaceModelPoseGraph &)>;

/// \brief Interface element representing a Model
class SDFORMAT_VISIBLE InterfaceModel
{
  /// \brief Constructor
  /// \param[in] name The *local* name (no nesting, e.g. "::").  If this name
  /// contains "::", an error will be raised.
  /// \param[in] _static Whether the model is static
  /// \param[in] _canonicalLinkName The canonical link's name. This is the
  /// resolved name of the canonical link, therefore, it cannot be an empty
  /// string. The link must be added to the model. If the canonical link is
  /// nested in a child model, this should be the relative name (using the "::"
  /// delimiter) of the canonical link in the scope of this model.
  /// \param[in] _poseInParentFrame Model frame pose relative to the parent
  /// frame. Defaults to identity.
  /// \note This will not be used if //include/pose is specified.
  public: InterfaceModel(const std::string &_name,
              const sdf::RepostureFunction &_repostureFunction,
              bool _static,
              const std::string &_canonicalLinkName,
              const ignition::math::Pose3d &_poseInParentFrame = {});

  /// \brief Get the name of the model.
  /// \return Local name of the model.
  public: const std::string &Name() const;

  /// \brief Get whether the model is static.
  /// \return Whether the model is static.
  public: bool Static() const;

  /// \brief Get the canonical link name.
  /// \remark Unlike Model::CanonicalLinkName which simply returns
  /// the value of //model/@canonical_link without resolving to an actual link,
  /// this function returns the resolved canonical link name.
  /// \return Canonical link name of the model.
  public: const std::string &CanonicalLinkName() const;

  /// \brief Get the pose of this model in the parent frame.
  /// \return Pose of this model in the parent model frame.
  public: const ignition::math::Pose3d &ModelFramePoseInParentFrame() const;

  /// \brief Provided so that hierarchy can still be leveraged from SDFormat.
  /// \param[in] _nestedModel A child interface model.
  public: void AddNestedModel(sdf::InterfaceModelConstPtr _nestedModel);

  /// \brief Gets registered nested models.
  public: const std::vector<sdf::InterfaceModelConstPtr> &NestedModels() const;

  /// \brief Add an interface frame. Provided so that the including SDFormat
  /// model can still interface with the declared frames.
  /// \param[in] _frame A child interface frame.
  public: void AddFrame(sdf::InterfaceFrame _frame);

  /// \brief Gets registered frames.
  public: const std::vector<sdf::InterfaceFrame> &Frames() const;

  /// \brief Add an interface joint. Provided so that the including SDFormat
  /// model can still interface with the declared joints.
  /// \param[in] _joint A child interface joint.
  public: void AddJoint(sdf::InterfaceJoint _joint);

  /// \brief Gets registered joints.
  public: const std::vector<sdf::InterfaceJoint> &Joints() const;

  /// \brief Add an interface link. Provided so that the including SDFormat
  /// model can still interface with the declared links.
  /// \param[in] _link A child interface link.
  public: void AddLink(sdf::InterfaceLink _link);

  /// \brief Gets registered links.
  public: const std::vector<sdf::InterfaceLink> &Links() const;

  /// \brief Whether the custom parser supports merge-include.
  /// \return True if the custom parser supports merge-include
  public: bool ParserSupportsMergeInclude() const;

  /// \brief Set whether the custom parser supports merge-include.
  /// \brief[in] _val True if the custom parser supports merge-include.
  public: void SetParserSupportsMergeInclude(bool _val);

  /// \brief Recursively invoke the reposture callback if a the callback is set.
  /// \param[in] _poseGraph Object used for resolving poses.
  /// \param[in] _name Override name of graph scope.
  private: void InvokeRepostureFunction(
               sdf::ScopedGraph<PoseRelativeToGraph> _graph,
               const std::optional<std::string> &_name) const;

  friend World;
  friend Model;
  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
