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
#ifndef SDF_LINK_HH_
#define SDF_LINK_HH_

#include <memory>
#include <optional>
#include <string>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
#include "sdf/system_util.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Error.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class Collision;
  class Light;
  class ParserConfig;
  class ParticleEmitter;
  class Projector;
  class Sensor;
  class Visual;
  struct PoseRelativeToGraph;
  template <typename T> class ScopedGraph;

  class SDFORMAT_VISIBLE Link
  {
    /// \brief Default constructor
    public: Link();

    /// \brief Load the link based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the link based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \param[in] _config Parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Get the name of the link.
    /// The name of a link must be unique within the scope of a Model.
    /// \return Name of the link.
    public: std::string Name() const;

    /// \brief Set the name of the link.
    /// The name of a link must be unique within the scope of a Model.
    /// \param[in] _name Name of the link.
    public: void SetName(const std::string &_name);

    /// \brief Get the density of the inertial if it has been set.
    /// \return Density of the inertial if it has been set,
    /// otherwise std::nullopt.
    public: std::optional<double> Density() const;

    /// \brief Set the density of the inertial.
    /// \param[in] _density Density of the inertial.
    public: void SetDensity(double _density);

    /// \brief Get the ElementPtr to the <auto_inertia_params> element
    /// This element can be used as a parent element to hold user-defined
    /// params for the custom moment of inertia calculator.
    /// \return ElementPtr object for the <auto_inertia_params> element.
    public: sdf::ElementPtr AutoInertiaParams() const;

    /// \brief Function to set the auto inertia params using a
    /// sdf ElementPtr object
    /// \param[in] _autoInertiaParams ElementPtr to <auto_inertia_params>
    /// element
    public: void SetAutoInertiaParams(const sdf::ElementPtr _autoInertiaParams);

    /// \brief Get the number of visuals.
    /// \return Number of visuals contained in this Link object.
    public: uint64_t VisualCount() const;

    /// \brief Get a visual based on an index.
    /// \param[in] _index Index of the visual. The index should be in the
    /// range [0..VisualCount()).
    /// \return Pointer to the visual. Nullptr if the index does not exist.
    /// \sa uint64_t VisualCount() const
    public: const Visual *VisualByIndex(const uint64_t _index) const;

    /// \brief Get a mutable visual based on an index.
    /// \param[in] _index Index of the visual. The index should be in the
    /// range [0..VisualCount()).
    /// \return Pointer to the visual. Nullptr if the index does not exist.
    /// \sa uint64_t VisualCount() const
    public: Visual *VisualByIndex(uint64_t _index);

    /// \brief Get whether a visual name exists.
    /// \param[in] _name Name of the visual to check.
    /// \return True if there exists a visual with the given name.
    public: bool VisualNameExists(const std::string &_name) const;

    /// \brief Get a visual based on a name.
    /// \param[in] _name Name of the visual.
    /// \return Pointer to the visual. Nullptr if the name does not exist.
    public: const Visual *VisualByName(const std::string &_name) const;

    /// \brief Get a mutable visual based on a name.
    /// \param[in] _name Name of the visual.
    /// \return Pointer to the visual. Nullptr if the name does not exist.
    public: Visual *VisualByName(const std::string &_name);

    /// \brief Get the number of collisions.
    /// \return Number of collisions contained in this Link object.
    public: uint64_t CollisionCount() const;

    /// \brief Get a collision based on an index.
    /// \param[in] _index Index of the collision. The index should be in the
    /// range [0..CollisionCount()).
    /// \return Pointer to the collision. Nullptr if the index does not exist.
    /// \sa uint64_t CollisionCount() const
    public: const Collision *CollisionByIndex(const uint64_t _index) const;

    /// \brief Get a mutable collision based on an index.
    /// \param[in] _index Index of the collision. The index should be in the
    /// range [0..CollisionCount()).
    /// \return Pointer to the collision. Nullptr if the index does not exist.
    /// \sa uint64_t CollisionCount() const
    public: Collision *CollisionByIndex(uint64_t _index);

    /// \brief Get whether a collision name exists.
    /// \param[in] _name Name of the collision to check.
    /// \return True if there exists a collision with the given name.
    public: bool CollisionNameExists(const std::string &_name) const;

    /// \brief Get a collision based on a name.
    /// \param[in] _name Name of the collision.
    /// \return Pointer to the collision. Nullptr if the name does not exist.
    public: const Collision *CollisionByName(const std::string &_name) const;

    /// \brief Get a mutable collision based on a name.
    /// \param[in] _name Name of the collision.
    /// \return Pointer to the collision. Nullptr if the name does not exist.
    public: Collision *CollisionByName(const std::string &_name);

    /// \brief Get the number of lights.
    /// \return Number of lights contained in this Link object.
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

    /// \brief Get a light based on a name.
    /// \param[in] _name Name of the light.
    /// \return Pointer to the light. Nullptr if the name does not exist.
    public: const Light *LightByName(const std::string &_name) const;

    /// \brief Get a mutable light based on a name.
    /// \param[in] _name Name of the light.
    /// \return Pointer to the light. Nullptr if the name does not exist.
    public: Light *LightByName(const std::string &_name);

    /// \brief Get the number of sensors.
    /// \return Number of sensors contained in this Link object.
    public: uint64_t SensorCount() const;

    /// \brief Get a sensor based on an index.
    /// \param[in] _index Index of the sensor. The index should be in the
    /// range [0..SensorCount()).
    /// \return Pointer to the sensor. Nullptr if the index does not exist.
    /// \sa uint64_t SensorCount() const
    public: const Sensor *SensorByIndex(const uint64_t _index) const;

    /// \brief Get a mutable sensor based on an index.
    /// \param[in] _index Index of the sensor. The index should be in the
    /// range [0..SensorCount()).
    /// \return Pointer to the sensor. Nullptr if the index does not exist.
    /// \sa uint64_t SensorCount() const
    public: Sensor *SensorByIndex(uint64_t _index);

    /// \brief Get whether a sensor name exists.
    /// \param[in] _name Name of the sensor to check.
    /// \return True if there exists a sensor with the given name.
    public: bool SensorNameExists(const std::string &_name) const;

    /// \brief Get a sensor based on a name.
    /// \param[in] _name Name of the sensor.
    /// \return Pointer to the sensor. Nullptr if a sensor with the given name
    ///  does not exist.
    /// \sa bool SensorNameExists(const std::string &_name) const
    public: const Sensor *SensorByName(const std::string &_name) const;

    /// \brief Get a mutable sensor based on a name.
    /// \param[in] _name Name of the sensor.
    /// \return Pointer to the sensor. Nullptr if a sensor with the given name
    ///  does not exist.
    /// \sa bool SensorNameExists(const std::string &_name) const
    public: Sensor *SensorByName(const std::string &_name);

    /// \brief Get the number of particle emitters.
    /// \return Number of particle emitters contained in this Link object.
    public: uint64_t ParticleEmitterCount() const;

    /// \brief Get a particle emitter based on an index.
    /// \param[in] _index Index of the particle emitter.
    /// The index should be in the range [0..ParticleEmitterCount()).
    /// \return Pointer to the particle emitter. Nullptr if the index does
    /// not exist.
    /// \sa uint64_t ParticleEmitterCount() const
    public: const ParticleEmitter *ParticleEmitterByIndex(
                const uint64_t _index) const;

    /// \brief Get a mutable particle emitter based on an index.
    /// \param[in] _index Index of the particle emitter.
    /// The index should be in the range [0..ParticleEmitterCount()).
    /// \return Pointer to the particle emitter. Nullptr if the index does
    /// not exist.
    /// \sa uint64_t ParticleEmitterCount() const
    public: ParticleEmitter *ParticleEmitterByIndex(uint64_t _index);

    /// \brief Get whether a particle emitter name exists.
    /// \param[in] _name Name of the particle emitter to check.
    /// \return True if there exists a particle emitter with the given name.
    public: bool ParticleEmitterNameExists(const std::string &_name) const;

    /// \brief Get a particle emitter based on a name.
    /// \param[in] _name Name of the particle emitter.
    /// \return Pointer to the particle emitter. Nullptr if a particle emitter
    /// with the given name does not exist.
    /// \sa bool ParticleEmitterNameExists(const std::string &_name) const
    public: const ParticleEmitter *ParticleEmitterByName(
                const std::string &_name) const;

    /// \brief Get a mutable particle emitter based on a name.
    /// \param[in] _name Name of the particle emitter.
    /// \return Pointer to the particle emitter. Nullptr if a particle emitter
    /// with the given name does not exist.
    /// \sa bool ParticleEmitterNameExists(const std::string &_name) const
    public: ParticleEmitter *ParticleEmitterByName(const std::string &_name);

    /// \brief Get the number of projectors.
    /// \return Number of projectors contained in this Link object.
    public: uint64_t ProjectorCount() const;

    /// \brief Get a projector based on an index.
    /// \param[in] _index Index of the projector.
    /// The index should be in the range [0..ProjectorCount()).
    /// \return Pointer to the projector. Nullptr if the index does
    /// not exist.
    /// \sa uint64_t ProjectorCount() const
    public: const Projector *ProjectorByIndex(
                const uint64_t _index) const;

    /// \brief Get a mutable projector based on an index.
    /// \param[in] _index Index of the projector.
    /// The index should be in the range [0..ProjectorCount()).
    /// \return Pointer to the projector. Nullptr if the index does
    /// not exist.
    /// \sa uint64_t ProjectorCount() const
    public: Projector *ProjectorByIndex(uint64_t _index);

    /// \brief Get whether a projector name exists.
    /// \param[in] _name Name of the projector to check.
    /// \return True if there exists a projector with the given name.
    public: bool ProjectorNameExists(const std::string &_name) const;

    /// \brief Get a projector based on a name.
    /// \param[in] _name Name of the projector.
    /// \return Pointer to the projector. Nullptr if a projector
    /// with the given name does not exist.
    /// \sa bool ProjectorNameExists(const std::string &_name) const
    public: const Projector *ProjectorByName(
                const std::string &_name) const;

    /// \brief Get a mutable projector based on a name.
    /// \param[in] _name Name of the projector.
    /// \return Pointer to the projector. Nullptr if a projector
    /// with the given name does not exist.
    /// \sa bool ProjectorNameExists(const std::string &_name) const
    public: Projector *ProjectorByName(const std::string &_name);

    /// \brief Get the inertial value for this link. The inertial object
    /// consists of the link's mass, a 3x3 rotational inertia matrix, and
    /// a pose for the inertial reference frame. The units for mass is
    /// kilograms with a default value of 1kg. The 3x3 rotational inertia
    /// matrix is symmetric and only 6 above-diagonal elements of this matrix
    /// are specified the Interial's gz::math::MassMatrix3 property.
    ///
    /// The origin of the inertial reference frame needs to be at the center
    /// of mass expressed in this link's frame.
    /// The axes of the inertial reference frame do not need to
    /// be aligned with the principal axes of the inertia.
    /// \return The link's inertial value.
    /// \sa void SetInertial(const gz::math::Inertiald &_inertial)
    public: const gz::math::Inertiald &Inertial() const;

    /// \brief Set the inertial value for this link.
    /// \param[in] _inertial The link's inertial value.
    /// \return True if the inertial is valid, false otherwise.
    /// \sa const gz::math::Inertiald &Inertial() const
    public: bool SetInertial(const gz::math::Inertiald &_inertial);

    /// \brief Resolve the Inertial to a specified frame.
    /// If there are any errors resolving the Inertial, the output will not
    /// be modified.
    /// \param[out] _inertial The resolved Inertial.
    /// \param[in] _resolveTo The Inertial will be resolved with respect to this
    /// frame. If unset or empty, the default resolve-to frame will be used.
    /// \return Errors in resolving pose.
    public: Errors ResolveInertial(gz::math::Inertiald &_inertial,
                                   const std::string &_resolveTo = "") const;

    /// \brief Calculate & set inertial values(mass, mass matrix
    /// & inertial pose) for the link. This function will calculate
    /// the inertial values if the auto attribute is set to true.
    /// If mass is not provided by the user, the inertial values will be
    /// determined based on either link density or collision density if
    /// explicitly set. Otherwise, if mass is specified, the inertia matrix
    /// will be scaled to match the desired mass, while respecting
    /// the ratio of density values between collisions.
    /// \param[out] _errors A vector of Errors object. Each object
    /// would contain an error code and an error message.
    /// \param[in] _config Custom parser configuration
    public: void ResolveAutoInertials(sdf::Errors &_errors,
                                    const ParserConfig &_config);

    /// \brief Get the pose of the link. This is the pose of the link
    /// as specified in SDF (<link> <pose> ... </pose></link>).
    /// \return The pose of the link.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the link.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The new link pose.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent model.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent model.
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

    /// \brief Give the scoped PoseRelativeToGraph to be used for resolving
    /// poses. This is private and is intended to be called by Model::Load.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Allow Model::Load to call SetPoseRelativeToGraph.
    friend class Model;

    /// \brief Check if this link should be subject to wind.
    /// If true, this link should be affected by wind.
    /// \return true if the model should be subject to wind, false otherwise.
    /// \sa bool Model::EnableWind
    public: bool EnableWind() const;

    /// \brief Check if this link should be subject to gravity.
    /// If true, this link should be affected by gravity.
    /// \return true if the model should be subject to gravity, false otherwise.
    public: bool EnableGravity() const;

    /// \brief Set whether this link should be subject to wind.
    /// \param[in] _enableWind True or false depending on whether the link
    /// should be subject to wind.
    /// \sa Model::SetEnableWind(bool)
    public: void SetEnableWind(bool _enableWind);

    /// \brief Set whether this link should be subject to gravity.
    /// \param[in] _enableGravity True or false depending on whether the link
    /// should be subject to gravity.
    public: void SetEnableGravity(bool _enableGravity);

    /// \brief Check if this link is kinematic only
    /// \return true if the link is kinematic only, false otherwise.
    public: bool Kinematic() const;

    /// \brief Set whether this link is kinematic only.
    /// \param[in] _kinematic True to make the link kinematic only,
    public: void SetKinematic(bool _kinematic);

    /// \brief Check if the automatic calculation for the link inertial
    /// is enabled or not.
    /// \return True if automatic calculation is enabled. This can be done
    /// setting the auto attribute of the <inertial> element of the link to
    /// true or by setting the autoInertia member to true
    /// using SetAutoInertia().
    public: bool AutoInertia() const;

    /// \brief Enable automatic inertial calculations by setting autoInertia
    /// to true.
    /// \param[in] _autoInertia True or False
    public: void SetAutoInertia(bool _autoInertia);

    /// \brief Check if the inertial values for this link were saved.
    /// If true, the inertial values for this link wont be calculated
    /// when CalculateInertial() is called. This value is set to true
    /// when CalculateInertial() is called with SAVE_CALCULATION
    /// configuration.
    /// \return True if CalculateInertial() was called with SAVE_CALCULATION
    /// configuration, false otherwise.
    public: bool AutoInertiaSaved() const;

    /// \brief Set the autoInertiaSaved() values
    /// \param[in] _autoInertiaSaved True or False
    public: void SetAutoInertiaSaved(bool _autoInertiaSaved);

    /// \brief Add a collision to the link.
    /// \param[in] _collision Collision to add.
    /// \return True if successful, false if a collision with the name already
    /// exists.
    public: bool AddCollision(const Collision &_collision);

    /// \brief Add a visual to the link.
    /// \param[in] _visual Visual to add.
    /// \return True if successful, false if a visual with the name already
    /// exists.
    public: bool AddVisual(const Visual &_visual);

    /// \brief Add a light to the link.
    /// \param[in] _light Light to add.
    /// \return True if successful, false if a light with the name already
    /// exists.
    public: bool AddLight(const Light &_light);

    /// \brief Add a sensor to the link.
    /// \param[in] _sensor Sensor to add.
    /// \return True if successful, false if a sensor with the name already
    /// exists.
    public: bool AddSensor(const Sensor &_sensor);

    /// \brief Add a particle emitter to the link.
    /// \param[in] _emitter Particle emitter to add.
    /// \return True if successful, false if a particle emitter with the name
    /// already exists.
    public: bool AddParticleEmitter(const ParticleEmitter &_emitter);

    /// \brief Add a projector to the link.
    /// \param[in] _projector Projector to add.
    /// \return True if successful, false if a projector with the name
    /// already exists.
    public: bool AddProjector(const Projector &_projector);

    /// \brief Remove all collisions
    public: void ClearCollisions();

    /// \brief Remove all visuals
    public: void ClearVisuals();

    /// \brief Remove all lights
    public: void ClearLights();

    /// \brief Remove all sensors
    public: void ClearSensors();

    /// \brief Remove all particle emitters
    public: void ClearParticleEmitters();

    /// \brief Remove all projectors
    public: void ClearProjectors();

    /// \brief Create and return an SDF element filled with data from this
    /// link.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated link values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
