/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include <ignition/math/Pose3.hh>
#include <ignition/math/SemanticVersion.hh>
#include "sdf/Error.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::ModelPrivate
{
  /// \brief Name of the model.
  public: std::string name = "";

  /// \brief True if this model is specified as static, false otherwise.
  public: bool isStatic = false;

  /// \brief True if this model should self-collide, false otherwise.
  public: bool selfCollide = false;

  /// \brief True if this model is allowed to conserve processing power by not
  /// updating when it's at rest.
  public: bool allowAutoDisable = true;

  /// \brief True if this model should be subject to wind, false otherwise.
  public: bool enableWind = false;

  /// \brief Name of the canonical link.
  public: std::string canonicalLink = "";

  /// \brief Name of the placement frame
  public: std::string placementFrameName = "";

  /// \brief Pose of the model
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The links specified in this model.
  public: std::vector<Link> links;

  /// \brief The joints specified in this model.
  public: std::vector<Joint> joints;

  /// \brief The frames specified in this model.
  public: std::vector<Frame> frames;

  /// \brief The nested models specified in this model.
  public: std::vector<Model> models;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Frame Attached-To Graph constructed during Load.
  public: std::shared_ptr<sdf::FrameAttachedToGraph> frameAttachedToGraph;

  /// \brief Pose Relative-To Graph constructed during Load.
  public: std::shared_ptr<sdf::PoseRelativeToGraph> poseGraph;

  /// \brief Pose Relative-To Graph in parent (world or __model__) scope.
  public: std::weak_ptr<const sdf::PoseRelativeToGraph> parentPoseGraph;

  /// \brief Scope name of parent Pose Relative-To Graph (world or __model__).
  public: std::string parentPoseGraphScopeName;
};

/////////////////////////////////////////////////
Model::Model()
  : dataPtr(new ModelPrivate)
{
}

/////////////////////////////////////////////////
Model::~Model()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Model::Model(const Model &_model)
  : dataPtr(new ModelPrivate(*_model.dataPtr))
{
  if (_model.dataPtr->frameAttachedToGraph)
  {
    this->dataPtr->frameAttachedToGraph =
        std::make_shared<sdf::FrameAttachedToGraph>(
            *_model.dataPtr->frameAttachedToGraph);
  }
  if (_model.dataPtr->poseGraph)
  {
    this->dataPtr->poseGraph = std::make_shared<sdf::PoseRelativeToGraph>(
        *_model.dataPtr->poseGraph);
  }
  for (auto &link : this->dataPtr->links)
  {
    link.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }
  for (auto &model : this->dataPtr->models)
  {
    model.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }
  for (auto &joint : this->dataPtr->joints)
  {
    joint.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
    joint.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
    frame.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }
}

/////////////////////////////////////////////////
Model::Model(Model &&_model) noexcept
  : dataPtr(std::exchange(_model.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Model &Model::operator=(const Model &_model)
{
  return *this = Model(_model);
}

/////////////////////////////////////////////////
Model &Model::operator=(Model &&_model)
{
  std::swap(this->dataPtr, _model.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors Model::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;
  ignition::math::SemanticVersion sdfVersion(_sdf->OriginalVersion());

  // Check that the provided SDF element is a <model>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "model")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Model, but the provided SDF element is not a "
        "<model>."});
    return errors;
  }

  // Read the models's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A model name is required, but the name is not set."});
  }

  // Check that the model's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied model name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // Read the model's canonical_link attribute
  if (_sdf->HasAttribute("canonical_link"))
  {
    auto pair = _sdf->Get<std::string>("canonical_link", "");
    if (pair.second)
    {
      this->dataPtr->canonicalLink = pair.first;
    }
  }

  this->dataPtr->placementFrameName = _sdf->Get<std::string>("placement_frame",
                             this->dataPtr->placementFrameName).first;

  this->dataPtr->isStatic = _sdf->Get<bool>("static", false).first;

  this->dataPtr->selfCollide = _sdf->Get<bool>("self_collide", false).first;

  this->dataPtr->allowAutoDisable =
    _sdf->Get<bool>("allow_auto_disable", true).first;

  this->dataPtr->enableWind = _sdf->Get<bool>("enable_wind", false).first;

  // Load the pose. Ignore the return value since the model pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  if (!_sdf->HasUniqueChildNames())
  {
    sdfwarn << "Non-unique names detected in XML children of model with name["
            << this->Name() << "].\n";
  }

  // Set of implicit and explicit frame names in this model for tracking
  // name collisions
  std::unordered_set<std::string> frameNames;

  // Load nested models.
  Errors nestedModelLoadErrors = loadUniqueRepeated<Model>(_sdf, "model",
    this->dataPtr->models);
  errors.insert(errors.end(),
                nestedModelLoadErrors.begin(),
                nestedModelLoadErrors.end());

  // Nested models are loaded first, and loadUniqueRepeated ensures there are no
  // duplicate names, so these names can be added to frameNames without
  // checking uniqueness.
  for (const auto &model : this->dataPtr->models)
  {
    frameNames.insert(model.Name());
  }

  // Load all the links.
  Errors linkLoadErrors = loadUniqueRepeated<Link>(_sdf, "link",
    this->dataPtr->links);
  errors.insert(errors.end(), linkLoadErrors.begin(), linkLoadErrors.end());

  // Check links for name collisions and modify and warn if so.
  for (auto &link : this->dataPtr->links)
  {
    std::string linkName = link.Name();
    if (frameNames.count(linkName) > 0)
    {
      // This link has a name collision
      if (sdfVersion < ignition::math::SemanticVersion(1, 7))
      {
        // This came from an old file, so try to workaround by renaming link
        linkName += "_link";
        int i = 0;
        while (frameNames.count(linkName) > 0)
        {
          linkName = link.Name() + "_link" + std::to_string(i++);
        }
        sdfwarn << "Link with name [" << link.Name() << "] "
                << "in model with name [" << this->Name() << "] "
                << "has a name collision, changing link name to ["
                << linkName << "].\n";
        link.SetName(linkName);
      }
      else
      {
        sdferr << "Link with name [" << link.Name() << "] "
               << "in model with name [" << this->Name() << "] "
               << "has a name collision. Please rename this link.\n";
      }
    }
    frameNames.insert(linkName);
  }

  // If the model is not static and has no nested models:
  // Require at least one link so the implicit model frame can be attached to
  // something.
  if (!this->Static() && this->dataPtr->links.empty() &&
      this->dataPtr->models.empty())
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                     "A model must have at least one link."});
  }

  // Load all the joints.
  Errors jointLoadErrors = loadUniqueRepeated<Joint>(_sdf, "joint",
    this->dataPtr->joints);
  errors.insert(errors.end(), jointLoadErrors.begin(), jointLoadErrors.end());

  // Check joints for name collisions and modify and warn if so.
  for (auto &joint : this->dataPtr->joints)
  {
    std::string jointName = joint.Name();
    if (frameNames.count(jointName) > 0)
    {
      // This joint has a name collision
      if (sdfVersion < ignition::math::SemanticVersion(1, 7))
      {
        // This came from an old file, so try to workaround by renaming joint
        jointName += "_joint";
        int i = 0;
        while (frameNames.count(jointName) > 0)
        {
          jointName = joint.Name() + "_joint" + std::to_string(i++);
        }
        sdfwarn << "Joint with name [" << joint.Name() << "] "
                << "in model with name [" << this->Name() << "] "
                << "has a name collision, changing joint name to ["
                << jointName << "].\n";
        joint.SetName(jointName);
      }
      else
      {
        sdferr << "Joint with name [" << joint.Name() << "] "
               << "in model with name [" << this->Name() << "] "
               << "has a name collision. Please rename this joint.\n";
      }
    }
    frameNames.insert(jointName);
  }

  // Load all the frames.
  Errors frameLoadErrors = loadUniqueRepeated<Frame>(_sdf, "frame",
    this->dataPtr->frames);
  errors.insert(errors.end(), frameLoadErrors.begin(), frameLoadErrors.end());

  // Check frames for name collisions and modify and warn if so.
  for (auto &frame : this->dataPtr->frames)
  {
    std::string frameName = frame.Name();
    if (frameNames.count(frameName) > 0)
    {
      // This joint has a name collision
      if (sdfVersion < ignition::math::SemanticVersion(1, 7))
      {
        // This came from an old file, so try to workaround by renaming frame
        frameName += "_frame";
        int i = 0;
        while (frameNames.count(frameName) > 0)
        {
          frameName = frame.Name() + "_frame" + std::to_string(i++);
        }
        sdfwarn << "Frame with name [" << frame.Name() << "] "
                << "in model with name [" << this->Name() << "] "
                << "has a name collision, changing frame name to ["
                << frameName << "].\n";
        frame.SetName(frameName);
      }
      else
      {
        sdferr << "Frame with name [" << frame.Name() << "] "
               << "in model with name [" << this->Name() << "] "
               << "has a name collision. Please rename this frame.\n";
      }
    }
    frameNames.insert(frameName);
  }

  // Build the graphs.

  // Build the FrameAttachedToGraph if the model is not static.
  // Re-enable this when the buildFrameAttachedToGraph implementation handles
  // static models.
  if (!this->Static())
  {
    this->dataPtr->frameAttachedToGraph
        = std::make_shared<FrameAttachedToGraph>();
    Errors frameAttachedToGraphErrors =
    buildFrameAttachedToGraph(*this->dataPtr->frameAttachedToGraph, this);
    errors.insert(errors.end(), frameAttachedToGraphErrors.begin(),
                                frameAttachedToGraphErrors.end());
    Errors validateFrameAttachedGraphErrors =
      validateFrameAttachedToGraph(*this->dataPtr->frameAttachedToGraph);
    errors.insert(errors.end(), validateFrameAttachedGraphErrors.begin(),
                                validateFrameAttachedGraphErrors.end());
    for (auto &joint : this->dataPtr->joints)
    {
      joint.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
    }
    for (auto &frame : this->dataPtr->frames)
    {
      frame.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
    }
  }

  // Build the PoseRelativeToGraph
  this->dataPtr->poseGraph = std::make_shared<PoseRelativeToGraph>();
  Errors poseGraphErrors =
  buildPoseRelativeToGraph(*this->dataPtr->poseGraph, this);
  errors.insert(errors.end(), poseGraphErrors.begin(),
                              poseGraphErrors.end());
  Errors validatePoseGraphErrors =
    validatePoseRelativeToGraph(*this->dataPtr->poseGraph);
  errors.insert(errors.end(), validatePoseGraphErrors.begin(),
                              validatePoseGraphErrors.end());
  for (auto &link : this->dataPtr->links)
  {
    link.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }
  for (auto &model : this->dataPtr->models)
  {
    Errors setPoseRelativeToGraphErrors =
      model.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
    errors.insert(errors.end(), setPoseRelativeToGraphErrors.begin(),
                                setPoseRelativeToGraphErrors.end());
  }
  for (auto &joint : this->dataPtr->joints)
  {
    joint.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetPoseRelativeToGraph(this->dataPtr->poseGraph);
  }

  // Update the model pose to account for the placement frame.
  if (!this->dataPtr->placementFrameName.empty())
  {
    ignition::math::Pose3d X_MPf;

    sdf::Errors resolveErrors =
        sdf::resolvePose(X_MPf, *this->dataPtr->poseGraph,
            this->dataPtr->placementFrameName, "__model__");
    if (resolveErrors.empty())
    {
      // Before this update, i.e, as specified in the SDFormat, the model pose
      // (X_RPf) is the pose of the placement frame (Pf) relative to a frame (R)
      // in the parent scope of the model. However, when this model (M) is
      // inserted into a pose graph of the parent scope, only the pose (X_RM)
      // of the __model__ frame can be used. Thus, the model pose has to be
      // updated to X_RM.
      //
      // Note that X_RPf is the raw pose specified in //model/pose before this
      // update.
      const auto &X_RPf = this->dataPtr->pose;
      auto &X_RM = this->dataPtr->pose;
      X_RM = X_RPf * X_MPf.Inverse();
    }
    else
    {
      errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());
    }
  }

  // The placement_frame attributes of included child models are currently lost
  // during parsing because the parser expands the included models into the
  // parent model. As a temporary workardound to preserve this information, the
  // parser injects <model name>::__placement_frame__ for every included child
  // model. This serves as an identifier that the model frame of the child model
  // should be treated as if it has its placement_frame attribute set.
  // This will not be necessary when included nested models work as directly
  // nested models. See
  // https://github.com/osrf/sdformat/issues/319#issuecomment-665214004
  // TODO (addisu) Remove placementFrameIdentifier once PR addressing
  // https://github.com/osrf/sdformat/issues/284 lands
  for (auto &frame : this->dataPtr->frames)
  {
    auto placementSubstrInd = frame.Name().rfind("::__placement_frame__");
    if (placementSubstrInd != std::string::npos)
    {
      const std::string childModelName =
          frame.Name().substr(0, placementSubstrInd);
      // Find the model frame associated with this placement frame
      const Frame *childModelFrame =
          this->FrameByName(childModelName + "::__model__");
      if (nullptr != childModelFrame)
      {
        // The RawPose of the child model frame is the desired pose of the
        // placement frame relative to a frame (R) in the scope of the parent
        // model. We don't need to resolve the pose because the relative_to
        // frame remains unchanged after the pose update here. i.e, the updated
        // pose of the child model frame will still be relative to R.
        const auto &X_RPf = childModelFrame->RawPose();

        ignition::math::Pose3d X_MPf;
        sdf::Errors resolveErrors =
            frame.SemanticPose().Resolve(X_MPf, childModelFrame->Name());
        errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

        // We need to update childModelFrame's pose relative to the parent
        // frame as well as the corresponding edge in the pose graph because
        // just updating childModelFrame doesn't update the pose graph.
        auto X_RM = X_RPf * X_MPf.Inverse();
        frame.SetRawPose(X_RM);
        sdf::updateGraphPose(
            *this->dataPtr->poseGraph, childModelFrame->Name(), X_RM);
      }
      else
      {
        errors.push_back({ErrorCode::MODEL_PLACEMENT_FRAME_INVALID,
            "Found a __placement_frame__ for child model with name [" +
            childModelName + "], but the model does not exist in the" +
            " parent model with name [" + this->Name() + "]"});
      }
    }
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Model::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Model::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
bool Model::Static() const
{
  return this->dataPtr->isStatic;
}

/////////////////////////////////////////////////
void Model::SetStatic(const bool _static)
{
  this->dataPtr->isStatic = _static;
}

/////////////////////////////////////////////////
bool Model::SelfCollide() const
{
  return this->dataPtr->selfCollide;
}

/////////////////////////////////////////////////
void Model::SetSelfCollide(const bool _selfCollide)
{
  this->dataPtr->selfCollide = _selfCollide;
}

/////////////////////////////////////////////////
bool Model::AllowAutoDisable() const
{
  return this->dataPtr->allowAutoDisable;
}

/////////////////////////////////////////////////
void Model::SetAllowAutoDisable(const bool _allowAutoDisable)
{
  this->dataPtr->allowAutoDisable = _allowAutoDisable;
}

/////////////////////////////////////////////////
bool Model::EnableWind() const
{
  return this->dataPtr->enableWind;
}

/////////////////////////////////////////////////
void Model::SetEnableWind(const bool _enableWind)
{
  this->dataPtr->enableWind =_enableWind;
}

/////////////////////////////////////////////////
uint64_t Model::LinkCount() const
{
  return this->dataPtr->links.size();
}

/////////////////////////////////////////////////
const Link *Model::LinkByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->links.size())
    return &this->dataPtr->links[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::LinkNameExists(const std::string &_name) const
{
  for (auto const &l : this->dataPtr->links)
  {
    if (l.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Model::JointCount() const
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
const Joint *Model::JointByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->joints.size())
    return &this->dataPtr->joints[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::JointNameExists(const std::string &_name) const
{
  for (auto const &j : this->dataPtr->joints)
  {
    if (j.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Joint *Model::JointByName(const std::string &_name) const
{
  for (auto const &j : this->dataPtr->joints)
  {
    if (j.Name() == _name)
    {
      return &j;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t Model::FrameCount() const
{
  return this->dataPtr->frames.size();
}

/////////////////////////////////////////////////
const Frame *Model::FrameByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->frames.size())
    return &this->dataPtr->frames[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::FrameNameExists(const std::string &_name) const
{
  for (auto const &f : this->dataPtr->frames)
  {
    if (f.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Frame *Model::FrameByName(const std::string &_name) const
{
  for (auto const &f : this->dataPtr->frames)
  {
    if (f.Name() == _name)
    {
      return &f;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t Model::ModelCount() const
{
  return this->dataPtr->models.size();
}

/////////////////////////////////////////////////
const Model *Model::ModelByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->models.size())
    return &this->dataPtr->models[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::ModelNameExists(const std::string &_name) const
{
  for (auto const &m : this->dataPtr->models)
  {
    if (m.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Model *Model::ModelByName(const std::string &_name) const
{
  for (auto const &m : this->dataPtr->models)
  {
    if (m.Name() == _name)
    {
      return &m;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
const Link *Model::CanonicalLink() const
{
  return this->CanonicalLinkAndRelativeName().first;
}

/////////////////////////////////////////////////
std::pair<const Link*, std::string> Model::CanonicalLinkAndRelativeName() const
{
  if (this->CanonicalLinkName().empty())
  {
    if (this->LinkCount() > 0)
    {
      auto firstLink = this->LinkByIndex(0);
      return std::make_pair(firstLink, firstLink->Name());
    }
    else if (this->ModelCount() > 0)
    {
      // Recursively choose the canonical link of the first nested model
      // (depth first search).
      auto firstModel = this->ModelByIndex(0);
      auto canonicalLinkAndName = firstModel->CanonicalLinkAndRelativeName();
      // Prepend firstModelName if a valid link is found.
      if (nullptr != canonicalLinkAndName.first)
      {
        canonicalLinkAndName.second =
            firstModel->Name() + "::" + canonicalLinkAndName.second;
      }
      return canonicalLinkAndName;
    }
    else
    {
      return std::make_pair(nullptr, "");
    }
  }
  else
  {
    return std::make_pair(this->LinkByName(this->CanonicalLinkName()),
                          this->CanonicalLinkName());
  }
}

/////////////////////////////////////////////////
const std::string &Model::CanonicalLinkName() const
{
  return this->dataPtr->canonicalLink;
}

/////////////////////////////////////////////////
void Model::SetCanonicalLinkName(const std::string &_canonicalLink)
{
  this->dataPtr->canonicalLink = _canonicalLink;
}

/////////////////////////////////////////////////
const std::string &Model::PlacementFrameName() const
{
  return this->dataPtr->placementFrameName;
}

/////////////////////////////////////////////////
void Model::SetPlacementFrameName(const std::string &_placementFrame)
{
  this->dataPtr->placementFrameName = _placementFrame;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Model::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Model::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Model::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Model::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
Errors Model::SetPoseRelativeToGraph(
    std::weak_ptr<const PoseRelativeToGraph> _graph)
{
  Errors errors;

  auto graph = _graph.lock();
  if (!graph)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "Tried to set PoseRelativeToGraph with invalid pointer."});
    return errors;
  }

  this->dataPtr->parentPoseGraphScopeName = graph->sourceName;
  this->dataPtr->parentPoseGraph = _graph;

  return errors;
}

/////////////////////////////////////////////////
sdf::SemanticPose Model::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->parentPoseGraphScopeName,
      this->dataPtr->parentPoseGraph);
}

/////////////////////////////////////////////////
const Link *Model::LinkByName(const std::string &_name) const
{
  for (auto const &l : this->dataPtr->links)
  {
    if (l.Name() == _name)
    {
      return &l;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
sdf::ElementPtr Model::Element() const
{
  return this->dataPtr->sdf;
}
