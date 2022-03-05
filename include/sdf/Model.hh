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
#ifndef SDF_MODEL_HH_
#define SDF_MODEL_HH_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/Plugin.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class Frame;
  class InterfaceModel;
  class Joint;
  class Link;
  class ParserConfig;
  class NestedInclude;
  struct PoseRelativeToGraph;
  struct FrameAttachedToGraph;
  template <typename T> class ScopedGraph;
  using InterfaceModelConstPtr = std::shared_ptr<const InterfaceModel>;


  class SDFORMAT_VISIBLE Model
  {
    /// \brief Default constructor
    public: Model();

    /// \brief Load the model based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the model based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \param[in] _config Parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(sdf::ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Check that the FrameAttachedToGraph and PoseRelativeToGraph
    /// are valid.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors ValidateGraphs() const;

    /// \brief Get the name of the model.
    /// The name of the model should be unique within the scope of a World.
    /// \return Name of the model.
    public: std::string Name() const;

    /// \brief Set the name of the model.
    /// The name of the model should be unique within the scope of a World.
    /// \param[in] _name Name of the model.
    public: void SetName(const std::string &_name);

    /// \brief Check if this model should be static.
    /// A static model is one that is not subject to physical forces (in other
    /// words, it's purely kinematic instead of dynamic).
    /// \return true if the model should be static, false if it is dynamic.
    public: bool Static() const;

    /// \brief Set this model to be static or not static.
    /// \param[in] _static True or false depending on whether the model should
    /// be static.
    /// \sa Static()
    public: void SetStatic(bool _static);

    /// \brief Check if this model should self-collide.
    /// A self-colliding model is a model whose links will collide if they come
    /// into contact. If a model is not self-colliding, its links will pass
    /// through each other.
    /// \return true if the model should self-collide, false otherwise.
    public: bool SelfCollide() const;

    /// \brief Set this model to self-collide or not self-collide.
    /// \param[in] _selfCollide True or false depending on whether the model
    /// should self-collide.
    /// \sa SelfCollide()
    public: void SetSelfCollide(bool _selfCollide);

    /// \brief Check if this model should be allowed to auto-disable.
    /// If auto-disable is allowed, a model that is at rest can choose to not
    /// update its dynamics.
    /// \return true if auto-disable is allowed for this model
    public: bool AllowAutoDisable() const;

    /// \brief Set this model to allow auto-disabling.
    /// \param[in] _allowAutoDisable True or false depending on whether the
    /// model should be allowed to auto-disable.
    /// \sa AllowAutoDisable()
    public: void SetAllowAutoDisable(bool _allowAutoDisable);

    /// \brief Check if this model should be subject to wind.
    /// If true, all links in the model should be affected by the wind. This can
    /// be overridden per link.
    /// \return true if the model should be subject to wind, false otherwise.
    public: bool EnableWind() const;

    /// \brief Set whether this model should be subject to wind.
    /// \param[in] _enableWind True or false depending on whether the model
    /// should be subject to wind.
    public: void SetEnableWind(bool _enableWind);

    /// \brief Get the number of links that are immediate (not nested) children
    /// of this Model object.
    /// \remark LinkByName() can find links that are not immediate children of
    /// this Model object.
    /// \return Number of links contained in this Model object.
    public: uint64_t LinkCount() const;

    /// \brief Get an immediate (not nested) child link based on an index.
    /// \param[in] _index Index of the link. The index should be in the
    /// range [0..LinkCount()).
    /// \return Pointer to the link. Nullptr if the index does not exist.
    /// \sa uint64_t LinkCount() const
    public: const Link *LinkByIndex(const uint64_t _index) const;

    /// \brief Get an immediate (not nested) mutable child link based on an
    /// index.
    /// \param[in] _index Index of the link. The index should be in the
    /// range [0..LinkCount()).
    /// \return Pointer to the link. Nullptr if the index does not exist.
    /// \sa uint64_t LinkCount() const
    public: Link *LinkByIndex(uint64_t _index);

    /// \brief Get a link based on a name.
    /// \param[in] _name Name of the link.
    /// To get a link in a nested model, prefix the link name with the
    /// sequence of nested models containing this link, delimited by "::".
    /// \return Pointer to the link. Nullptr if the name does not exist.
    public: const Link *LinkByName(const std::string &_name) const;

    /// \brief Get a mutable link based on a name.
    /// \param[in] _name Name of the link.
    /// To get a link in a nested model, prefix the link name with the
    /// sequence of nested models containing this link, delimited by "::".
    /// \return Pointer to the link. Nullptr if the name does not exist.
    public: Link *LinkByName(const std::string &_name);

    /// \brief Get whether a link name exists.
    /// \param[in] _name Name of the link to check.
    /// To check for a link in a nested model, prefix the link name with
    /// the sequence of nested models containing this link, delimited by "::".
    /// \return True if there exists a link with the given name.
    public: bool LinkNameExists(const std::string &_name) const;

    /// \brief Get the number of joints that are immediate (not nested) children
    /// of this Model object.
    /// \remark JointByName() can find joints that are not immediate children of
    /// this Model object.
    /// \return Number of joints contained in this Model object.
    public: uint64_t JointCount() const;

    /// \brief Get an immediate (not nested) child joint based on an index.
    /// \param[in] _index Index of the joint. The index should be in the
    /// range [0..JointCount()).
    /// \return Pointer to the joint. Nullptr if the index does not exist.
    /// \sa uint64_t JointCount() const
    public: const Joint *JointByIndex(const uint64_t _index) const;

    /// \brief Get an immediate (not nested) mutable child joint based on an
    /// index.
    /// \param[in] _index Index of the joint. The index should be in the
    /// range [0..JointCount()).
    /// \return Pointer to the joint. Nullptr if the index does not exist.
    /// \sa uint64_t JointCount() const
    public: Joint *JointByIndex(uint64_t _index);

    /// \brief Get whether a joint name exists.
    /// \param[in] _name Name of the joint to check.
    /// To check for a joint in a nested model, prefix the joint name with
    /// the sequence of nested models containing this joint, delimited by "::".
    /// \return True if there exists a joint with the given name.
    public: bool JointNameExists(const std::string &_name) const;

    /// \brief Get a joint based on a name.
    /// \param[in] _name Name of the joint.
    /// To get a joint in a nested model, prefix the joint name with the
    /// sequence of nested models containing this joint, delimited by "::".
    /// \return Pointer to the joint. Nullptr if a joint with the given name
    ///  does not exist.
    /// \sa bool JointNameExists(const std::string &_name) const
    public: const Joint *JointByName(const std::string &_name) const;

    /// \brief Get a mubtable joint based on a name.
    /// \param[in] _name Name of the joint.
    /// To get a joint in a nested model, prefix the joint name with the
    /// sequence of nested models containing this joint, delimited by "::".
    /// \return Pointer to the joint. Nullptr if a joint with the given name
    ///  does not exist.
    /// \sa bool JointNameExists(const std::string &_name) const
    public: Joint *JointByName(const std::string &_name);

    /// \brief Get the number of explicit frames that are immediate (not nested)
    /// children of this Model object.
    /// \remark FrameByName() can find explicit frames that are not immediate
    /// children of this Model object.
    /// \return Number of explicit frames contained in this Model object.
    public: uint64_t FrameCount() const;

    /// \brief Get an immediate (not nested) child explicit frame based on an
    /// index.
    /// \param[in] _index Index of the explicit frame. The index should be in
    /// the range [0..FrameCount()).
    /// \return Pointer to the explicit frame. Nullptr if the index does not
    /// exist.
    /// \sa uint64_t FrameCount() const
    public: const Frame *FrameByIndex(const uint64_t _index) const;

    /// \brief Get a mutable immediate (not nested) child explicit frame based
    /// on an index.
    /// \param[in] _index Index of the explicit frame. The index should be in
    /// the range [0..FrameCount()).
    /// \return Pointer to the explicit frame. Nullptr if the index does not
    /// exist.
    /// \sa uint64_t FrameCount() const
    public: Frame *FrameByIndex(uint64_t _index);

    /// \brief Get an explicit frame based on a name.
    /// \param[in] _name Name of the explicit frame.
    /// To get a frame in a nested model, prefix the frame name with the
    /// sequence of nested models containing this frame, delimited by "::".
    /// \return Pointer to the explicit frame. Nullptr if the name does not
    /// exist.
    public: const Frame *FrameByName(const std::string &_name) const;

    /// \brief Get a mutable explicit frame based on a name.
    /// \param[in] _name Name of the explicit frame.
    /// To get a frame in a nested model, prefix the frame name with the
    /// sequence of nested models containing this frame, delimited by "::".
    /// \return Pointer to the explicit frame. Nullptr if the name does not
    /// exist.
    public: Frame *FrameByName(const std::string &_name);

    /// \brief Get whether an explicit frame name exists.
    /// \param[in] _name Name of the explicit frame to check.
    /// To check for a frame in a nested model, prefix the frame name with
    /// the sequence of nested models containing this frame, delimited by "::".
    /// \return True if there exists an explicit frame with the given name.
    public: bool FrameNameExists(const std::string &_name) const;

    /// \brief Get the number of nested models that are immediate (not
    /// recursively nested) children of this Model object.
    /// \remark ModelByName() can find nested models that are not immediate
    /// children of this Model object.
    /// \return Number of nested models contained in this Model object.
    public: uint64_t ModelCount() const;

    /// \brief Get an immediate (not recursively nested) child model based on an
    /// index.
    /// \param[in] _index Index of the nested model. The index should be in the
    /// range [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: const Model *ModelByIndex(const uint64_t _index) const;

    /// \brief Get an immediate (not recursively nested) mutable child model
    // based on an index.
    /// \param[in] _index Index of the nested model. The index should be in the
    /// range [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: Model *ModelByIndex(uint64_t _index);

    /// \brief Get whether a nested model name exists.
    /// \param[in] _name Name of the nested model to check.
    /// To check for a model nested in other models, prefix the model name
    /// with the sequence of nested model names, delimited by "::".
    /// \return True if there exists a nested model with the given name.
    public: bool ModelNameExists(const std::string &_name) const;

    /// \brief Get a nested model based on a name.
    /// \param[in] _name Name of the nested model.
    /// To get a model nested in other models, prefix the model name
    /// with the sequence of nested model names, delimited by "::".
    /// \return Pointer to the model. Nullptr if a model with the given name
    ///  does not exist.
    /// \sa bool ModelNameExists(const std::string &_name) const
    public: const Model *ModelByName(const std::string &_name) const;

    /// \brief Get a mutable nested model based on a name.
    /// \param[in] _name Name of the nested model.
    /// To get a model nested in other models, prefix the model name
    /// with the sequence of nested model names, delimited by "::".
    /// \return Pointer to the model. Nullptr if a model with the given name
    ///  does not exist.
    /// \sa bool ModelNameExists(const std::string &_name) const
    public: Model *ModelByName(const std::string &_name);

    /// \brief Get the pose of the model. This is the pose of the model
    /// as specified in SDF (<model> <pose> ... </pose></model>), and is
    /// typically used to express the position and rotation of a model in a
    /// global coordinate frame.
    /// \return The pose of the model.
    public: const ignition::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the model.
    /// \sa const ignition::math::Pose3d &RawPose() const
    /// \param[in] _pose The new model pose.
    public: void SetRawPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the model's canonical link
    /// \return An immutable pointer to the canonical link
    public: const Link *CanonicalLink() const;

    /// \brief Get the name of the model's canonical link. An empty value
    /// indicates that the first link in the model or the first link found
    /// in a depth first search of nested models is the canonical link.
    /// \return The name of the canonical link.
    public: const std::string &CanonicalLinkName() const;

    /// \brief Set the name of the model's canonical link. An empty value
    /// indicates that the first link in the model or the first link found
    /// in a depth first search of nested models is the canonical link.
    /// \param[in] _canonicalLink The name of the canonical link.
    public: void SetCanonicalLinkName(const std::string &_canonicalLink);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent model/world coordinate frame.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent model/world coordinate frame.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get SemanticPose object of this object to aid in resolving
    /// poses.
    /// \return SemanticPose object for this link.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Get the name of the placement frame of the model.
    /// \return Name of the placement frame attribute of the model.
    public: const std::string &PlacementFrameName() const;

    /// \brief Set the name of the placement frame of the model.
    /// The specified placement frame must exist within the model.
    /// \param[in] _name Name of the placement frame.
    public: void SetPlacementFrameName(const std::string &_name);

    /// \brief Get the model's canonical link and the nested name of the link
    /// relative to the current model, delimited by "::".
    /// \return An immutable pointer to the canonical link and the nested
    /// name of the link relative to the current model.
    // TODO(addisu): If the canonical link is inside an interface model, this
    // function returns {nullptr, name}. This can be problematic for downstream
    // applications.
    public: std::pair<const Link *, std::string> CanonicalLinkAndRelativeName()
        const;

    /// \brief Get the number of nested interface models that are immediate (not
    /// recursively nested) children of this Model object.
    /// \return Number of nested interface models contained in this Model
    /// object.
    public: uint64_t InterfaceModelCount() const;

    /// \brief Get an immediate (not recursively nested) child interface model
    /// based on an index.
    /// \param[in] _index Index of the nested interface model. The index should
    /// be in the range [0..InterfaceModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t InterfaceModelCount() const
    public: std::shared_ptr<const InterfaceModel> InterfaceModelByIndex(
                const uint64_t _index) const;

    /// \brief Get the nested include information of an immediate (not
    /// recursively nested) child interface model based on an index.
    /// \param[in] _index Index of the nested interface model. The index should
    /// be in the range [0..InterfaceModelCount()).
    /// \return Pointer to the nested include information. Nullptr if the index
    /// does not exist.
    /// \sa uint64_t InterfaceModelCount() const
    public: const NestedInclude *InterfaceModelNestedIncludeByIndex(
                const uint64_t _index) const;

    /// \brief Create and return an SDF element filled with data from this
    /// model.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[in] _useIncludeTag When true, the model's URI is used to create
    /// an SDF `<include>` rather than a `<model>`. The model's URI must be
    /// first set using the `Model::SetUri` function. If the model's URI is
    /// empty, then a `<model>` element will be generated. The default is true
    /// so that URI values are used when ToElement is called from a
    /// World object. Make sure to use `Model::SetUri` even when the model
    /// is loaded from an `<include>` tag since the parser will
    /// automatically expand an `<include>` element to a `<model>` element.
    /// \return SDF element pointer with updated model values.
    public: sdf::ElementPtr ToElement(bool _useIncludeTag = true) const;

    /// \brief Check if a given name exists in the FrameAttachedTo graph at the
    /// scope of the model.
    /// \param[in] _name Name of the implicit or explicit frame to check.
    /// To check for a frame in a nested model, prefix the frame name with
    /// the sequence of nested models containing this frame, delimited by "::".
    /// \return True if the frame name is found in the FrameAttachedTo graph.
    /// False otherwise, or if the frame graph is invalid.
    /// \note This function assumes the model has a valid FrameAttachedTo graph.
    /// It will return false if the graph is invalid.
    public: bool NameExistsInFrameAttachedToGraph(
                const std::string &_name) const;

    /// \brief Add a link to the model.
    /// \param[in] _link Link to add.
    /// \return True if successful, false if a link with the name already
    /// exists.
    public: bool AddLink(const Link &_link);

    /// \brief Add a joint to the model.
    /// \param[in] _link Joint to add.
    /// \return True if successful, false if a joint with the name already
    /// exists.
    public: bool AddJoint(const Joint &_joint);

    /// \brief Add a model to the model.
    /// \param[in] _model Model to add.
    /// \return True if successful, false if a model with the name already
    /// exists.
    public: bool AddModel(const Model &_model);

    /// \brief Add a frame to the model.
    /// \param[in] _frame Frame to add.
    /// \return True if successful, false if a frame with the name already
    /// exists.
    public: bool AddFrame(const Frame &_frame);

    /// \brief Remove all links.
    public: void ClearLinks();

    /// \brief Remove all joints.
    public: void ClearJoints();

    /// \brief Remove all models.
    public: void ClearModels();

    /// \brief Remove all frames.
    public: void ClearFrames();

    /// \brief Get the URI associated with this model
    /// \return The model's URI, or empty string if it has not been set.
    public: std::string Uri() const;

    /// \brief Set the URI associated with this model.
    /// \param[in] _uri The model's URI.
    public: void SetUri(const std::string &_uri);

    /// \brief Get the plugins attached to this object.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: const sdf::Plugins &Plugins() const;

    /// \brief Get a mutable vector of plugins attached to this object.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: sdf::Plugins &Plugins();

    /// \brief Remove all plugins
    public: void ClearPlugins();

    /// \brief Add a plugin to this object.
    /// \param[in] _plugin Plugin to add.
    public: void AddPlugin(const Plugin &_plugin);

    /// \brief Give the scoped PoseRelativeToGraph to be used for resolving
    /// poses. This is private and is intended to be called by Root::Load or
    /// World::SetPoseRelativeToGraph if this is a standalone model and
    /// Model::SetPoseRelativeToGraph if this is a nested model.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Give the scoped FrameAttachedToGraph to be used for resolving
    /// attached bodies. This is private and is intended to be called by
    /// Root::Load or World::SetFrameAttachedToGraph if this is a standalone
    /// model and Model::SetFrameAttachedToGraph if this is a nested model.
    /// \param[in] _graph scoped FrameAttachedToGraph object.
    private: void SetFrameAttachedToGraph(
        sdf::ScopedGraph<FrameAttachedToGraph> _graph);

    /// \brief Get the list of merged interface models.
    /// \return The list of merged interface models.
    private: const std::vector<std::pair<std::optional<sdf::NestedInclude>,
             sdf::InterfaceModelConstPtr>> &MergedInterfaceModels() const;

    /// \brief Allow Root::Load, World::SetPoseRelativeToGraph, or
    /// World::SetFrameAttachedToGraph to call SetPoseRelativeToGraph and
    /// SetFrameAttachedToGraph
    friend class Root;
    friend class World;

    // Allow ModelWrapper from FrameSemantics.cc to call MergedInterfaceModels
    friend struct ModelWrapper;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
