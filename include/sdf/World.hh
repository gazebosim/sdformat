/*
 * Copyright 2017 Open Source Robotics Foundation
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
#ifndef SDF_WORLD_HH_
#define SDF_WORLD_HH_

#include <memory>
#include <optional>
#include <string>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/Atmosphere.hh"
#include "sdf/Element.hh"
#include "sdf/Gui.hh"
#include "sdf/Plugin.hh"
#include "sdf/Scene.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class Actor;
  class Frame;
  class InterfaceModel;
  class Light;
  class Model;
  class ParserConfig;
  class Physics;
  class NestedInclude;
  struct PoseRelativeToGraph;
  struct FrameAttachedToGraph;
  template <typename T> class ScopedGraph;

  class SDFORMAT_VISIBLE World
  {
    /// \brief Default constructor
    public: World();

    /// \brief Load the world based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the world based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \param[in] _config Custom parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(sdf::ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Check that the FrameAttachedToGraph and PoseRelativeToGraph
    /// are valid.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors ValidateGraphs() const;

    /// \brief Get the name of the world.
    /// \return Name of the world.
    public: std::string Name() const;

    /// \brief Set the name of the world.
    /// \param[in] _name Name of the world.
    public: void SetName(const std::string &_name);

    /// \brief Get the audio device name. The audio device can be used to
    /// playback audio files. A value of "default" or an empty string
    /// indicates that the system's default audio device should be used.
    /// \return Audio device name.
    public: std::string AudioDevice() const;

    /// \brief Set the audio device name. See std::string AudioDevice() const
    /// for more information.
    /// \param[in] _device The new audio device name.
    /// \sa std::string AudioDevice() const
    public: void SetAudioDevice(const std::string &_device);

    /// \brief Get the wind linear velocity in the global/world coordinate
    /// frame. Units are meters per second \f$(\frac{m}{s})\f$
    /// \return Linear velocity of wind in the global/world coordinate frame.
    /// \sa void SetWindLinearVelocity(const ignition::math::Vector3d &_wind)
    public: ignition::math::Vector3d WindLinearVelocity() const;

    /// \brief Set the wind linear velocity in the global/world coordinate
    /// frame. Units are meters per second \f$(\frac{m}{s})\f$
    /// \param[in] _wind The new linear velocity of wind.
    /// \sa ignition::math::Vector3d WindLinearVelocity() const
    public: void SetWindLinearVelocity(const ignition::math::Vector3d &_wind);

    /// \brief Get the acceleration due to gravity. The default value is
    /// Earth's standard gravity at sea level, which equals
    /// [0, 0, -9.80665] \f$(\frac{m}{s^2})\f$
    /// \return Gravity vector in meters per second squared
    /// \f$(\frac{m}{s^2})\f$
    public: ignition::math::Vector3d Gravity() const;

    /// \brief Set the acceleration due to gravity. Units are meters per
    /// second squared \f$(\frac{m}{s^2})\f$
    /// \param[in] _gravity The new gravity vector.
    public: void SetGravity(const ignition::math::Vector3d &_gravity);

    /// \brief Get the magnetic vector in Tesla, expressed in
    /// a coordinate frame defined by the SphericalCoordinates property.
    /// A spherical coordinate can be specified in SDF using the
    /// <spherical_coordinates> element.
    /// \return Magnetic field vector.
    /// \sa SphericalCoordinates
    public: ignition::math::Vector3d MagneticField() const;

    /// \brief Set the magnetic vector in Tesla, expressed in
    /// a coordinate frame defined by the SphericalCoordinate.
    /// A spherical coordinate can be specified in SDF using the
    /// <spherical_coordinates> element.
    /// \param[in] _mag The new magnetic field vector.
    /// \sa SphericalCoordinates
    public: void SetMagneticField(const ignition::math::Vector3d &_mag);

    /// \brief Get the spherical coordinates for the world origin.
    /// \return Spherical coordinates or null if not defined.
    public: const ignition::math::SphericalCoordinates *
        SphericalCoordinates() const;

    /// \brief Set the spherical coordinates for the world origin.
    /// \param[in] _coord The new coordinates for the world origin.
    public: void SetSphericalCoordinates(
        const ignition::math::SphericalCoordinates &_coord);

    /// \brief Get the number of models that are immediate (not nested) children
    /// of this World object.
    /// \remark ModelByName() can find nested models that are not immediate
    /// children of this World object.
    /// \return Number of models contained in this World object.
    public: uint64_t ModelCount() const;

    /// \brief Get an immediate (not recursively nested) child model based on an
    /// index.
    /// \param[in] _index Index of the model. The index should be in the range
    /// [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: const Model *ModelByIndex(const uint64_t _index) const;

    /// \brief Get an immediate (not recursively nested) mutable child model
    /// based on an index.
    /// \param[in] _index Index of the model. The index should be in the range
    /// [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: Model *ModelByIndex(uint64_t _index);

    /// \brief Get a model based on a name.
    /// \param[in] _name Name of the model.
    /// To get a model nested in other models, prefix the model name
    /// with the sequence of nested model names, delimited by "::".
    /// \return Pointer to the model. Nullptr if a model with the given name
    /// does not exist.
    /// \sa bool ModelNameExists(const std::string &_name) const
    public: const Model *ModelByName(const std::string &_name) const;

    /// \brief Get a mutable model based on a name.
    /// \param[in] _name Name of the model.
    /// To get a model nested in other models, prefix the model name
    /// with the sequence of nested model names, delimited by "::".
    /// \return Pointer to the model. Nullptr if a model with the given name
    /// does not exist.
    /// \sa bool ModelNameExists(const std::string &_name) const
    public: Model *ModelByName(const std::string &_name);

    /// \brief Get whether a model name exists.
    /// \param[in] _name Name of the model to check.
    /// To check for a model nested in other models, prefix the model name with
    /// the sequence of nested models containing this model, delimited by "::".
    /// \return True if there exists a model with the given name.
    public: bool ModelNameExists(const std::string &_name) const;

    /// \brief Add a model to the world.
    /// \param[in] _model Model to add.
    /// \return True if successful, false if a model with the name already
    /// exists.
    public: bool AddModel(const Model &_model);

    /// \brief Add an actor to the world.
    /// \param[in] _actor Actor to add.
    /// \return True if successful, false if an actor with the name already
    /// exists.
    public: bool AddActor(const Actor &_actor);

    /// \brief Add a light to the world.
    /// \param[in] _light Light to add.
    /// \return True if successful, false if a light with the name already
    /// exists.
    public: bool AddLight(const Light &_light);

    /// \brief Add a physics object to the world.
    /// \param[in] _physics Physics to add.
    /// \return True if successful, false if a physics object with the name
    /// already exists.
    public: bool AddPhysics(const Physics &_physics);

    /// \brief Add a frame object to the world.
    /// \param[in] _frame Frame to add.
    /// \return True if successful, false if a frames object with the name
    /// already exists.
    public: bool AddFrame(const Frame &_frame);

    /// \brief Remove all models.
    public: void ClearModels();

    /// \brief Remove all models.
    public: void ClearActors();

    /// \brief Remove all models.
    public: void ClearLights();

    /// \brief Remove all physics.
    public: void ClearPhysics();

    /// \brief Remove all frames.
    public: void ClearFrames();

    /// \brief Get the number of actors.
    /// \return Number of actors contained in this World object.
    public: uint64_t ActorCount() const;

    /// \brief Get an actor based on an index.
    /// \param[in] _index Index of the actor. The index should be in the
    /// range [0..ActorCount()).
    /// \return Pointer to the actor. Nullptr if the index does not exist.
    /// \sa uint64_t ActorCount() const
    public: const Actor *ActorByIndex(const uint64_t _index) const;

    /// \brief Get a mutable actor based on an index.
    /// \param[in] _index Index of the actor. The index should be in the
    /// range [0..ActorCount()).
    /// \return Pointer to the actor. Nullptr if the index does not exist.
    /// \sa uint64_t ActorCount() const
    public: Actor *ActorByIndex(uint64_t _index);

    /// \brief Get whether an actor name exists.
    /// \param[in] _name Name of the actor to check.
    /// \return True if there exists an actor with the given name.
    public: bool ActorNameExists(const std::string &_name) const;

    /// \brief Get the number of explicit frames that are immediate (not nested)
    /// children of this World object.
    /// \remark FrameByName() can find explicit frames that are not immediate
    /// children of this World object.
    /// \return Number of explicit frames contained in this World object.
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

    /// \brief Get the number of lights.
    /// \return Number of lights contained in this World object.
    public: uint64_t LightCount() const;

    /// \brief Get a light based on an index.
    /// \param[in] _index Index of the light. The index should be in the
    /// range [0..LightCount()).
    /// \return Pointer to the light. Nullptr if the index does not exist.
    /// \sa uint64_t LightCount() const
    public: const Light *LightByIndex(const uint64_t _index) const;

    /// \brief Get a mutable light based on an index.
    /// \param[in] _index Index of the light. The index should be in the
    /// range [0..LightCount()).
    /// \return Pointer to the light. Nullptr if the index does not exist.
    /// \sa uint64_t LightCount() const
    public: Light *LightByIndex(uint64_t _index);

    /// \brief Get whether a light name exists.
    /// \param[in] _name Name of the light to check.
    /// \return True if there exists a light with the given name.
    public: bool LightNameExists(const std::string &_name) const;

    /// \brief Get a pointer to the atmosphere model associated with this
    /// world. A nullptr indicates that an atmosphere model has not been set.
    /// \return Pointer to this world's atmosphere model. Nullptr inidicates
    /// that there is no atmosphere model.
    public: const sdf::Atmosphere *Atmosphere() const;

    /// \brief Set the atmosphere model associated with this world.
    /// \param[in] _atmosphere The new atmosphere model for this world.
    public: void SetAtmosphere(const sdf::Atmosphere &_atmosphere);

    /// \brief Get a pointer to the Gui associated with this
    /// world. A nullptr indicates that a Gui element has not been specified.
    /// \return Pointer to this world's Gui parameters. Nullptr inidicates
    /// that there are no Gui parameters.
    public: const sdf::Gui *Gui() const;

    /// \brief Set the Gui parameters associated with this world.
    /// \param[in] _gui The new Gui parameter for this world
    public: void SetGui(const sdf::Gui &_gui);

    /// \brief Get a pointer to the Scene associated with this
    /// world. A nullptr indicates that a Scene element has not been specified.
    /// \return Pointer to this world's Scene parameters. Nullptr inidicates
    /// that there are no Scene parameters.
    public: const sdf::Scene *Scene() const;

    /// \brief Set the Scene parameters associated with this world.
    /// \param[in] _gui The new Scene parameter for this world
    public: void SetScene(const sdf::Scene &_scene);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the number of physics profiles.
    /// \return Number of physics profiles contained in this World object.
    public: uint64_t PhysicsCount() const;

    /// \brief Get a physics profile based on an index.
    /// \param[in] _index Index of the physics profile.
    /// The index should be in the range [0..PhysicsCount()).
    /// \return Pointer to the physics profile. Nullptr if the index does not
    /// exist.
    ///// \sa uint64_t PhysicsCount() const
    public: const Physics *PhysicsByIndex(const uint64_t _index) const;

    /// \brief Get a mutable physics profile based on an index.
    /// \param[in] _index Index of the physics profile.
    /// The index should be in the range [0..PhysicsCount()).
    /// \return Pointer to the physics profile. Nullptr if the index does not
    /// exist.
    ///// \sa uint64_t PhysicsCount() const
    public: Physics *PhysicsByIndex(uint64_t _index);

    /// \brief Get the default physics profile.
    /// \return Pointer to the default physics profile.
    public: const Physics *PhysicsDefault() const;

    /// \brief Get whether a physics profile name exists.
    /// \param[in] _name Name of the physics profile to check.
    /// \return True if there exists a physics profile with the given name.
    public: bool PhysicsNameExists(const std::string &_name) const;

    /// \brief Get the number of nested interface models that are immediate (not
    /// recursively nested) children of this World object.
    /// \return Number of nested interface models contained in this World
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
    public: const NestedInclude* InterfaceModelNestedIncludeByIndex(
                const uint64_t _index) const;

    /// \brief Create and return an SDF element filled with data from this
    /// world.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[in] _useIncludeTag This parameter is passed through to the
    /// Model::ToElement function.
    /// \return SDF element pointer with updated world values.
    public: sdf::ElementPtr ToElement(bool _useIncludeTag = true) const;

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

    /// \brief Give the Scoped PoseRelativeToGraph to be passed on to child
    /// entities for resolving poses. This is private and is intended to be
    /// called by Root::Load.
    /// \param[in] _graph Scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Give the Scoped FrameAttachedToGraph to be passed on to child
    /// entities for attached bodes. This is private and is intended to be
    /// called by Root::Load.
    /// \param[in] _graph Scoped FrameAttachedToGraph object.
    private: void SetFrameAttachedToGraph(
        sdf::ScopedGraph<FrameAttachedToGraph> _graph);

    /// \brief Allow Root::Load to call SetPoseRelativeToGraph and
    /// SetFrameAttachedToGraph
    friend class Root;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
