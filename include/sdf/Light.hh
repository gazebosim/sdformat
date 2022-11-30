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
#ifndef SDF_LIGHT_HH_
#define SDF_LIGHT_HH_

#include <memory>
#include <string>
#include <gz/math/Pose3.hh>
#include <gz/math/Angle.hh>

#include "sdf/Element.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class LightPrivate;
  struct PoseRelativeToGraph;

  /// \enum LightType
  /// \brief The set of light types. INVALID indicates that light type has
  /// not been set, or has not been set correctly.
  enum class LightType
  {
    /// \brief An invalid light. This indicates an error since a light type
    /// is required to fully specify a light.
    INVALID = 0,

    /// \brief A point light source.
    POINT = 1,

    /// \brief A spot light source.
    SPOT = 2,

    /// \brief A directional light source.
    DIRECTIONAL = 3,
  };

  /// \brief Provides a description of a light source. A light source can be
  /// point, spot, or directional light.
  class SDFORMAT_VISIBLE Light
  {
    /// \brief Default constructor
    public: Light();

    /// \brief Copy constructor
    /// \param[in] _light Light to copy.
    public: Light(const Light &_light);

    /// \brief Move constructor
    /// \param[in] _light Light to move.
    public: Light(Light &&_light) noexcept;

    /// \brief Destructor
    public: ~Light();

    /// \brief Move assignment operator.
    /// \param[in] _light Light to move.
    /// \return Reference to this.
    public: Light &operator=(Light &&_light);

    /// \brief Assignment operator.
    /// \param[in] _light The light to set values from.
    /// \return *this
    public: Light &operator=(const Light &_light);

    /// \brief Load the light based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the light type.
    /// \return The light type.
    public: LightType Type() const;

    /// \brief Set the light type.
    /// \param[in] _type The light type.
    public: void SetType(const LightType _type);

    /// \brief Get the name of the light.
    /// \return Name of the light.
    public: std::string Name() const;

    /// \brief Set the name of the light.
    /// \param[in] _name Name of the light.
    public: void SetName(const std::string &_name) const;

    /// \brief Get the pose of the light. This is the pose of the light
    /// as specified in SDF (<light> <pose> ... </pose></light>), and is
    /// typically used to express the position and rotation of a light in a
    /// global coordinate frame.
    /// \return The pose of the light.
    /// \deprecated See RawPose.
    public: const gz::math::Pose3d &Pose() const
        SDF_DEPRECATED(9.0);

    /// \brief Set the pose of the light.
    /// \sa const gz::math::Pose3d &Pose() const
    /// \param[in] _pose The new light pose.
    /// \deprecated See SetRawPose.
    public: void SetPose(const gz::math::Pose3d &_pose)
        SDF_DEPRECATED(9.0);

    /// \brief Get the pose of the light. This is the pose of the light
    /// as specified in SDF (<light> <pose> ... </pose></light>), and is
    /// typically used to express the position and rotation of a light in a
    /// global coordinate frame.
    /// \return The pose of the light.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the light.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The new light pose.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link/world coordinate frame.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent model/world coordinate frame.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get the name of the coordinate frame in which this light's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \return The name of the pose frame.
    /// \deprecated See PoseRelativeTo.
    public: const std::string &PoseFrame() const
        SDF_DEPRECATED(9.0);

    /// \brief Set the name of the coordinate frame in which this light's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \param[in] _frame The name of the pose frame.
    /// \deprecated See SetPoseRelativeTo.
    public: void SetPoseFrame(const std::string &_frame)
        SDF_DEPRECATED(9.0);

    /// \brief Get SemanticPose object of this object to aid in resolving
    /// poses.
    /// \return SemanticPose object for this link.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Get whether the light casts shadows.
    /// \return True if the light casts shadows.
    public: bool CastShadows() const;

    /// \brief Set whether the light casts shadows.
    /// \param[in] _cast True to indicate that the light casts shadows.
    public: void SetCastShadows(const bool _cast);

    /// \brief Get the diffuse color. The diffuse color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Diffuse color.
    public: gz::math::Color Diffuse() const;

    /// \brief Set the diffuse color. The diffuse color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Diffuse color.
    public: void SetDiffuse(const gz::math::Color &_color) const;

    /// \brief Get the specular color. The specular color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Specular color.
    public: gz::math::Color Specular() const;

    /// \brief Set the specular color. The specular color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Specular color.
    public: void SetSpecular(const gz::math::Color &_color) const;

    /// \brief Get the range of the light source in meters.
    /// \return Range of the light source in meters.
    public: double AttenuationRange() const;

    /// \brief Set the range of the light source in meters.
    /// \param[in] _factor Range of the light source in meters.
    public: void SetAttenuationRange(const double _range);

    /// \brief Get the linear attenuation factor. This value is clamped to
    /// a value between 0 and 1, where 1 means attenuate evenly over the
    /// distance.
    /// \return Linear attentuation factor.
    public: double LinearAttenuationFactor() const;

    /// \brief Set the linear attenuation factor. This value is clamped to
    /// a value between 0 and 1, where 1 means attenuate evenly over the
    /// distance.
    /// \param[in] _factor Linear attentuation factor.
    public: void SetLinearAttenuationFactor(const double _factor);

    /// \brief Get the constant attenuation factor. This value is clamped to
    /// a value between 0 and 1,  where 1.0 means never attenuate and 0.0 is
    /// complete attenutation.
    /// \return Constant attenuation factor.
    public: double ConstantAttenuationFactor() const;

    /// \brief Set the constant attenuation factor. This value is clamped to
    /// a value between 0 and 1,  where 1.0 means never attenuate and 0.0 is
    /// complete attenutation.
    /// \param[in] _factor Constant attenuation factor.
    public: void SetConstantAttenuationFactor(const double _factor);

    /// \brief Get the quadratic attenuation factor which adds a curvature to
    /// the attenuation.
    /// \return The quadratic attenuation factor.
    public: double QuadraticAttenuationFactor() const;

    /// \brief Set the quadratic attenuation factor which adds a curvature to
    /// the attenuation.
    /// \param[in] _factor The quadratic attenuation factor.
    public: void SetQuadraticAttenuationFactor(const double _factor);

    /// \brief Get the direction of the light source. This only has meaning
    /// for spot and directional light types. The default value is
    /// [0, 0, -1].
    /// \return Light source direction.
    public: gz::math::Vector3d Direction() const;

    /// \brief Set the direction of the light source. This only has meaning
    /// for spot and directional light types.
    /// \param[in] _dir Light source direction.
    public: void SetDirection(const gz::math::Vector3d  &_dir);

    /// \brief Get the angle covered by the bright inner cone.
    /// \return The angle covered by the bright inner cone.
    /// \note This function only has meaning for a spot light.
    public: gz::math::Angle SpotInnerAngle() const;

    /// \brief Set the angle covered by the bright inner cone.
    /// \param[in] _angle The angle covered by the bright inner cone.
    /// \note This function only has meaning for a spot light.
    public: void SetSpotInnerAngle(const gz::math::Angle &_angle);

    /// \brief Get the angle covered by the outer cone.
    /// \return The angle covered by the outer cone.
    /// \note This function only has meaning for a spot light.
    public: gz::math::Angle SpotOuterAngle() const;

    /// \brief Set the angle covered by the outer cone.
    /// \param[in] _angle The angle covered by the outer cone.
    /// \note This function only has meaning for a spot light.
    public: void SetSpotOuterAngle(const gz::math::Angle &_angle);

    /// \brief Get the rate of falloff between the inner and outer cones.
    /// A value of 1.0 is a linear falloff, less than 1.0 is a slower falloff,
    /// and a higher value indicates a faster falloff.
    /// \return The spot falloff.
    /// \note This function only has meaning for a spot light.
    public: double SpotFalloff() const;

    /// \brief Set the rate of falloff between the inner and outer cones.
    /// A value of 1.0 is a linear falloff, less than 1.0 is a slower falloff,
    /// and a higher value indicates a faster falloff.
    /// \param[in] _falloff The spot falloff.
    /// \note This function only has meaning for a spot light.
    public: void SetSpotFalloff(const double _falloff);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Helper function to copy from another light
    /// \param[in] _light Light to copy.
    private: void CopyFrom(const Light &_light);

    /// \brief Give the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph or World::Load.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Give a weak pointer to the PoseRelativeToGraph to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph or World::Load.
    /// \param[in] _graph Weak pointer to PoseRelativeToGraph.
    private: void SetPoseRelativeToGraph(
        std::weak_ptr<const PoseRelativeToGraph> _graph);

    /// \brief Allow Link::SetPoseRelativeToGraph or World::Load to call
    /// SetXmlParentName and SetPoseRelativeToGraph,
    /// but Link::SetPoseRelativeToGraph is a private function, so we need
    /// to befriend the entire class.
    friend class Link;
    friend class World;

    /// \brief Private data pointer.
    private: LightPrivate *dataPtr = nullptr;
  };
  }
}
#endif
