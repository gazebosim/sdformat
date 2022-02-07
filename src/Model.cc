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
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceLink.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/InterfaceModelPoseGraph.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"
#include "sdf/parser.hh"

using namespace sdf;

class sdf::Model::Implementation
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

  /// \brief The interface models specified in this model.
  public: std::vector<std::pair<std::optional<sdf::NestedInclude>,
          sdf::InterfaceModelConstPtr>> interfaceModels;

  /// \brief The interface models added into in this model via merge include.
  public: std::vector<std::pair<std::optional<sdf::NestedInclude>,
          sdf::InterfaceModelConstPtr>> mergedInterfaceModels;

  /// \brief The interface links specified in this model.
  public: std::vector<InterfaceLink> interfaceLinks;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Scoped Frame Attached-To graph at the parent model or world scope.
  public: sdf::ScopedGraph<sdf::FrameAttachedToGraph> frameAttachedToGraph;

  /// \brief Scoped Pose Relative-To graph at the parent model or world scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseGraph;

  /// \brief Scope name of parent Pose Relative-To Graph (world or __model__).
  public: std::string poseGraphScopeVertexName;

  /// \brief Optional URI string that specifies where this model was or
  /// can be loaded from.
  public: std::string uri = "";
};

/////////////////////////////////////////////////
Model::Model()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Model::Load(ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Model::Load(sdf::ElementPtr _sdf, const ParserConfig &_config)
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

  for (const auto &[name, size] :
       _sdf->CountNamedElements("", Element::NameUniquenessExceptions()))
  {
    if (size > 1)
    {
      sdfwarn << "Non-unique name[" << name << "] detected " << size
              << " times in XML children of model with name[" << this->Name()
              << "].\n";
    }
  }

  // Set of implicit and explicit frame names in this model for tracking
  // name collisions
  std::unordered_set<std::string> frameNames;

  // Load nested models.
  Errors nestedModelLoadErrors = loadUniqueRepeated<Model>(_sdf, "model",
    this->dataPtr->models, _config);
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

  // Load InterfaceModels into a temporary container so we can have special
  // handling for merged InterfaceModels.
  std::vector<std::pair<sdf::NestedInclude, sdf::InterfaceModelPtr>>
      tmpInterfaceModels;
  // Load included models via the interface API
  Errors interfaceModelLoadErrors = loadIncludedInterfaceModels(
      _sdf, _config, tmpInterfaceModels);
  errors.insert(errors.end(), interfaceModelLoadErrors.begin(),
      interfaceModelLoadErrors.end());

  for (const auto &[ifaceInclude, ifaceModel] : tmpInterfaceModels)
  {
    if (!ifaceInclude.IsMerge().value_or(false))
    {
      frameNames.insert(ifaceModel->Name());
      this->dataPtr->interfaceModels.emplace_back(ifaceInclude, ifaceModel);
    }
    else
    {
      const std::string proxyModelFrameName =
          computeMergedModelProxyFrameName(ifaceModel->Name());

      this->dataPtr->mergedInterfaceModels.emplace_back(ifaceInclude,
                                                        ifaceModel);

      // Copy interface links so that they can be used for resolving canonical
      // links.
      for (const auto &ifaceLink : ifaceModel->Links())
      {
        this->dataPtr->interfaceLinks.push_back(ifaceLink);
      }

      for (const auto &ifaceNestedModel : ifaceModel->NestedModels())
      {
        sdf::NestedInclude nestedInclude;
        // The pose of nested interface models is always given relative to the
        // parent model, but since this is a merged model, we set the relativeTo
        // to proxyModelFrameName. This nested interface models don't actually
        // have an //include that was used to nest them, so we create a
        // NestedInclude object and fill in the the necessary bits.
        nestedInclude.SetIncludePoseRelativeTo(proxyModelFrameName);
        nestedInclude.SetIncludeRawPose(
            ifaceNestedModel->ModelFramePoseInParentFrame());
        this->dataPtr->interfaceModels.emplace_back(nestedInclude,
                                                    ifaceNestedModel);
      }
    }
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
  // Require at least one (interface) link so the implicit model frame can be
  // attached to something.
  if (!this->Static() && this->dataPtr->links.empty() &&
      this->dataPtr->interfaceLinks.empty() && this->dataPtr->models.empty() &&
      this->dataPtr->interfaceModels.empty())
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


  return errors;
}

/////////////////////////////////////////////////
Errors Model::ValidateGraphs() const
{
  Errors errors =
      validateFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  Errors poseErrors =
      validatePoseRelativeToGraph(this->dataPtr->poseGraph);
  errors.insert(errors.end(), poseErrors.begin(), poseErrors.end());
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
  this->dataPtr->enableWind = _enableWind;
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
Link *Model::LinkByIndex(uint64_t _index)
{
  return const_cast<Link*>(
      static_cast<const Model*>(this)->LinkByIndex(_index));
}

/////////////////////////////////////////////////
bool Model::LinkNameExists(const std::string &_name) const
{
  return nullptr != this->LinkByName(_name);
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
Joint *Model::JointByIndex(uint64_t _index)
{
  return const_cast<Joint*>(
      static_cast<const Model*>(this)->JointByIndex(_index));
}

/////////////////////////////////////////////////
bool Model::JointNameExists(const std::string &_name) const
{
  return nullptr != this->JointByName(_name);
}

/////////////////////////////////////////////////
const Joint *Model::JointByName(const std::string &_name) const
{
  auto index = _name.rfind("::");
  if (index != std::string::npos)
  {
    const Model *model = this->ModelByName(_name.substr(0, index));
    if (nullptr != model)
    {
      return model->JointByName(_name.substr(index + 2));
    }

    // The nested model name preceding the last "::" could not be found.
    // For now, try to find a link that matches _name exactly.
    // When "::" are reserved and not allowed in names, then uncomment
    // the following line to return a nullptr.
    // return nullptr;
  }

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
Joint *Model::JointByName(const std::string &_name)
{
  return const_cast<Joint*>(
      static_cast<const Model*>(this)->JointByName(_name));
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
  return nullptr != this->FrameByName(_name);
}

/////////////////////////////////////////////////
const Frame *Model::FrameByName(const std::string &_name) const
{
  auto index = _name.rfind("::");
  if (index != std::string::npos)
  {
    const Model *model = this->ModelByName(_name.substr(0, index));
    if (nullptr != model)
    {
      return model->FrameByName(_name.substr(index + 2));
    }

    // The nested model name preceding the last "::" could not be found.
    // For now, try to find a link that matches _name exactly.
    // When "::" are reserved and not allowed in names, then uncomment
    // the following line to return a nullptr.
    // return nullptr;
  }

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
Model *Model::ModelByIndex(uint64_t _index)
{
  return const_cast<Model*>(
      static_cast<const Model*>(this)->ModelByIndex(_index));
}

/////////////////////////////////////////////////
bool Model::ModelNameExists(const std::string &_name) const
{
  return nullptr != this->ModelByName(_name);
}

/////////////////////////////////////////////////
const Model *Model::ModelByName(const std::string &_name) const
{
  auto index = _name.find("::");
  const std::string nextModelName = _name.substr(0, index);
  const Model *nextModel = nullptr;

  for (auto const &m : this->dataPtr->models)
  {
    if (m.Name() == nextModelName)
    {
      nextModel = &m;
      break;
    }
  }

  if (nullptr != nextModel && index != std::string::npos)
  {
    return nextModel->ModelByName(_name.substr(index + 2));
  }
  return nextModel;

}

/////////////////////////////////////////////////
Model *Model::ModelByName(const std::string &_name)
{
  return const_cast<Model*>(
      static_cast<const Model*>(this)->ModelByName(_name));
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
    if (this->dataPtr->interfaceLinks.size() > 0)
    {
      const auto &firstLink = this->dataPtr->interfaceLinks.front();
      return std::make_pair(nullptr, firstLink.Name());
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
    else if (this->InterfaceModelCount() > 0)
    {
      // Recursively choose the canonical link of the first nested model
      // (depth first search).
      auto firstModel = this->InterfaceModelByIndex(0);
      auto canonicalLinkName = firstModel->CanonicalLinkName();
      // Prepend firstModelName if a valid link is found.
      if (canonicalLinkName != "")
      {
        canonicalLinkName =
            sdf::JoinName(firstModel->Name(), canonicalLinkName);
      }
      return {nullptr, canonicalLinkName};
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
void Model::SetPoseRelativeToGraph(sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseGraph = _graph;
  this->dataPtr->poseGraphScopeVertexName =
      _graph.VertexLocalName(_graph.ScopeVertexId());

  auto childPoseGraph =
      this->dataPtr->poseGraph.ChildModelScope(this->Name());
  for (auto &model : this->dataPtr->models)
  {
    model.SetPoseRelativeToGraph(childPoseGraph);
  }
  for (auto &ifaceModelPair : this->dataPtr->interfaceModels)
  {
    // Don't invoke reposture for interface models that were merged because we
    // will reposture them below with a different scope in the poe graph.
    if (ifaceModelPair.first.has_value())
    {
      ifaceModelPair.second->InvokeRepostureFunction(childPoseGraph, {});
    }
  }
  for (auto &ifaceModelPair : this->dataPtr->mergedInterfaceModels)
  {
    ifaceModelPair.second->InvokeRepostureFunction(this->dataPtr->poseGraph,
                                                   this->Name());
  }
  for (auto &link : this->dataPtr->links)
  {
    link.SetPoseRelativeToGraph(childPoseGraph);
  }
  for (auto &joint : this->dataPtr->joints)
  {
    joint.SetPoseRelativeToGraph(childPoseGraph);
  }
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetPoseRelativeToGraph(childPoseGraph);
  }
}

/////////////////////////////////////////////////
void Model::SetFrameAttachedToGraph(
    sdf::ScopedGraph<FrameAttachedToGraph> _graph)
{
  this->dataPtr->frameAttachedToGraph = _graph;

  auto childFrameAttachedToGraph =
      this->dataPtr->frameAttachedToGraph.ChildModelScope(this->Name());
  for (auto &joint : this->dataPtr->joints)
  {
    joint.SetFrameAttachedToGraph(childFrameAttachedToGraph);
  }
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetFrameAttachedToGraph(childFrameAttachedToGraph);
  }
  for (auto &model : this->dataPtr->models)
  {
    model.SetFrameAttachedToGraph(childFrameAttachedToGraph);
  }
}

/////////////////////////////////////////////////
sdf::SemanticPose Model::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->name,
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->poseGraphScopeVertexName,
      this->dataPtr->poseGraph);
}

/////////////////////////////////////////////////
const Link *Model::LinkByName(const std::string &_name) const
{
  auto index = _name.rfind("::");
  if (index != std::string::npos)
  {
    const Model *model = this->ModelByName(_name.substr(0, index));
    if (nullptr != model)
    {
      return model->LinkByName(_name.substr(index + 2));
    }

    // The nested model name preceding the last "::" could not be found.
    // For now, try to find a link that matches _name exactly.
    // When "::" are reserved and not allowed in names, then uncomment
    // the following line to return a nullptr.
    // return nullptr;
  }

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
Link *Model::LinkByName(const std::string &_name)
{
  return const_cast<Link*>(
      static_cast<const Model*>(this)->LinkByName(_name));
}

/////////////////////////////////////////////////
sdf::ElementPtr Model::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
uint64_t Model::InterfaceModelCount() const
{
  return this->dataPtr->interfaceModels.size();
}

/////////////////////////////////////////////////
InterfaceModelConstPtr Model::InterfaceModelByIndex(
    const uint64_t _index) const
{
  if (_index < this->dataPtr->interfaceModels.size())
    return this->dataPtr->interfaceModels[_index].second;
  return nullptr;
}

/////////////////////////////////////////////////
const NestedInclude *Model::InterfaceModelNestedIncludeByIndex(
    const uint64_t _index) const
{
  if (_index < this->dataPtr->interfaceModels.size())
    return optionalToPointer(this->dataPtr->interfaceModels[_index].first);
  return nullptr;
}

/////////////////////////////////////////////////
const std::vector<
    std::pair<std::optional<sdf::NestedInclude>, sdf::InterfaceModelConstPtr>>
    &Model::MergedInterfaceModels() const
{
  return this->dataPtr->mergedInterfaceModels;
}

/////////////////////////////////////////////////
std::string Model::Uri() const
{
  return this->dataPtr->uri;
}

/////////////////////////////////////////////////
void Model::SetUri(const std::string &_uri)
{
  this->dataPtr->uri = _uri;
}

/////////////////////////////////////////////////
sdf::ElementPtr Model::ToElement(bool _useIncludeTag) const
{
  if (_useIncludeTag && !this->dataPtr->uri.empty())
  {
    sdf::ElementPtr worldElem(new sdf::Element);
    sdf::initFile("world.sdf", worldElem);

    sdf::ElementPtr includeElem = worldElem->AddElement("include");
    includeElem->GetElement("uri")->Set(this->Uri());
    includeElem->GetElement("name")->Set(this->Name());
    includeElem->GetElement("pose")->Set(this->RawPose());
    if (!this->dataPtr->poseRelativeTo.empty())
    {
      includeElem->GetElement("pose")->GetAttribute(
          "relative_to")->Set<std::string>(this->dataPtr->poseRelativeTo);
    }
    includeElem->GetElement("static")->Set(this->Static());
    includeElem->GetElement("placement_frame")->Set(this->PlacementFrameName());

    return includeElem;
  }

  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("model.sdf", elem);
  elem->GetAttribute("name")->Set(this->Name());

  if (!this->dataPtr->canonicalLink.empty())
  {
    elem->GetAttribute("canonical_link")->Set(this->dataPtr->canonicalLink);
  }

  if (!this->dataPtr->placementFrameName.empty())
  {
    elem->GetAttribute("placement_frame")->Set(
        this->dataPtr->placementFrameName);
  }

  elem->GetElement("static")->Set(this->Static());
  elem->GetElement("self_collide")->Set(this->SelfCollide());
  elem->GetElement("allow_auto_disable")->Set(this->AllowAutoDisable());
  elem->GetElement("enable_wind")->Set(this->EnableWind());

  // Set pose
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<ignition::math::Pose3d>(this->RawPose());

  // Links
  for (const sdf::Link &link : this->dataPtr->links)
    elem->InsertElement(link.ToElement(), true);

  // Joints
  for (const sdf::Joint &joint : this->dataPtr->joints)
    elem->InsertElement(joint.ToElement(), true);

  // Model
  for (const sdf::Model &model : this->dataPtr->models)
    elem->InsertElement(model.ToElement(_useIncludeTag), true);

  return elem;
}

//////////////////////////////////////////////////
bool Model::AddLink(const Link &_link)
{
  if (this->LinkNameExists(_link.Name()))
    return false;
  this->dataPtr->links.push_back(_link);
  return true;
}

//////////////////////////////////////////////////
bool Model::AddJoint(const Joint &_joint)
{
  if (this->JointNameExists(_joint.Name()))
    return false;
  this->dataPtr->joints.push_back(_joint);
  return true;
}

//////////////////////////////////////////////////
bool Model::AddModel(const Model &_model)
{
  if (this->ModelNameExists(_model.Name()))
    return false;
  this->dataPtr->models.push_back(_model);
  return true;
}

//////////////////////////////////////////////////
void Model::ClearLinks()
{
  this->dataPtr->links.clear();
}

//////////////////////////////////////////////////
void Model::ClearJoints()
{
  this->dataPtr->joints.clear();
}

//////////////////////////////////////////////////
void Model::ClearModels()
{
  this->dataPtr->models.clear();
}
