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
#ifndef SDF_JOINTAXIS_HH_
#define SDF_JOINTAXIS_HH_

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class JointAxisPrivate;

  /// \brief Parameters related to the axis of rotation for rotational joints,
  /// and the axis of translation for prismatic joints.
  class SDFORMAT_VISIBLE JointAxis
  {
    /// \brief Default constructor
    public: JointAxis();

    /// \brief Move constructor
    /// \param[in] _jointAxis Joint axis to move.
    public: JointAxis(JointAxis &&_jointAxis);

    /// \brief Destructor
    public: ~JointAxis();

    /// \brief Load the joint axis based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the initial joint position for this joint axis. The
    /// default value is zero (0.0).
    /// \return The initial joint position.
    /// \sa void SetInitialPosition(const double _pos)
    public: double InitialPosition() const;

    /// \brief Set the initial joint position for this joint axis.
    /// \param[in] _pos The initial joint position.
    /// \sa double InitialPosition() const
    public: void SetInitialPosition(const double _pos);

    /// \brief Get the x,y,z components of the axis unit vector.
    /// The axis is expressed in the joint frame unless UseParentModelFrame
    /// is true. The vector should be normalized.
    /// The default value is ignition::math::Vector3d::UnitZ which equals
    /// (0, 0, 1).
    /// \return The x,y,z components of the axis unit vector.
    /// \sa void SetXyz(const ignition::math::Vector3d &_xyz)
    public: ignition::math::Vector3d Xyz() const;

    /// \brief Set the x,y,z components of the axis unit vector.
    /// \param[in] _xyz The x,y,z components of the axis unit vector.
    /// \sa ignition::math::Vector3d Xyz() const
    public: void SetXyz(const ignition::math::Vector3d &_xyz);

    /// \brief Get whether to interpret the axis xyz value in the parent model
    /// frame instead of joint frame. The default value is false.
    /// \return True to interpret the axis xyz value in the parent model
    /// frame, false to use the joint frame.
    /// \sa void SetUseParentModelFrame(const bool _parentModelFrame)
    public: bool UseParentModelFrame() const;

    /// \brief Set whether to interpret the axis xyz value in the parent model
    /// instead of the joint frame.
    /// \param[in] _parentModelFrame True to interpret the axis xyz value in
    /// the parent model frame, false to use the joint frame.
    /// \sa bool UseParentModelFrame() const
    public: void SetUseParentModelFrame(const bool _parentModelFrame);

    /// \brief Get the physical velocity dependent viscous damping coefficient
    /// of the joint axis. The default value is zero (0.0).
    /// \return The physical velocity dependent viscous damping coefficient
    /// of the joint axis
    /// \sa void SetDamping(const double _damping)
    public: double Damping() const;

    /// \brief Set the physical velocity dependent viscous damping coefficient
    /// of the joint axis.
    /// \param[in] _damping The physical velocity dependent viscous damping
    /// coefficient of the joint axis
    /// \sa double Damping() const
    public: void SetDamping(const double _damping);

    /// \brief Get the physical static friction value of the joint. The
    /// default value is zero (0.0).
    /// \return The physical static friction value of the joint.
    /// \sa void SetFriction(const double _friction)
    public: double Friction() const;

    /// \brief Set the physical static friction value of the joint.
    /// \param[in] _friction The physical static friction value of the joint.
    /// \sa double Friction()
    public: void SetFriction(const double _friction);

    /// \brief Get the spring reference position for this joint axis. The
    /// default value is zero (0.0).
    /// \return The spring reference position for this joint axis.
    /// \sa void SetSpringReference(const double _spring)
    public: double SpringReference() const;

    /// \brief Set the spring reference position for this joint axis.
    /// \param[in] _spring The spring reference position for this joint axis.
    /// \sa double SpringReference() const
    public: void SetSpringReference(const double _spring);

    /// \brief Get the spring stiffness for this joint axis. The default
    /// value is zero (0.0).
    /// \return The spring stiffness for this joint axis.
    /// \sa void SetSpringStiffness(const double _spring)
    public: double SpringStiffness() const;

    /// \brief Set the spring stiffness for this joint axis.
    /// \param[in] _spring The spring stiffness for this joint axis.
    /// \sa double SpringStiffness() const
    public: void SetSpringStiffness(const double _spring);

    /// \brief Get the lower joint axis limit (radians for revolute joints,
    /// meters for prismatic joints). Not valid if the joint that uses this
    /// axis is continuous. The default value is -1e16.
    /// \return The lower joint axis limit
    /// \sa void SetLower(const double _lower)
    public: double Lower() const;

    /// \brief Set the lower joint axis limit (radians for revolute joints,
    /// meters for prismatic joints). Not valid if the joint that uses this
    /// axis is continuous.
    /// \param[in] _lower The lower joint axis limit
    /// \sa double Lower() const
    public: void SetLower(const double _lower);

    /// \brief Get the upper joint axis limit (radians for revolute joints,
    /// meters for prismatic joints). Not valid if joint that uses this
    /// axis is continuous. The default value is 1e16.
    /// \return The upper joint axis limit.
    /// \sa double SetUpper(const double _upper) const
    public: double Upper() const;

    /// \brief Set the upper joint axis limit (radians for revolute joints,
    /// meters for prismatic joints). Not valid if joint that uses this
    /// axis is continuous.
    /// \param[in] _upper The upper joint axis limit.
    /// \sa double Upper() const
    public: void SetUpper(const double _upper) const;

    /// \brief Get the value for enforcing the maximum joint effort applied.
    /// Limit is not enforced if value is negative. The default value is -1.
    /// \return Effort limit.
    /// \sa void SetEffort(double _effort)
    public: double Effort() const;

    /// \brief Set the value for enforcing the maximum joint effort applied.
    /// Limit is not enforced if value is negative.
    /// \param[in] _effort Effort limit.
    /// \sa double Effort() const
    public: void SetEffort(double _effort);

    /// \brief Get the value for enforcing the maximum joint velocity. The
    /// default value is -1.
    /// \return The value for enforcing the maximum joint velocity.
    /// \sa void SetVelocity(const double _velocity) const
    public: double MaxVelocity() const;

    /// \brief Set the value for enforcing the maximum joint velocity.
    /// \param[in] _velocity The value for enforcing the maximum joint velocity.
    /// \sa double Velocity() const
    public: void SetMaxVelocity(const double _velocity) const;

    /// \brief Get the joint stop stiffness. The default value is 1e8.
    /// \return The joint stop stiffness.
    /// \sa void SetStiffness(const double _stiffness) const
    public: double Stiffness() const;

    /// \brief Get the joint stop stiffness.
    /// \param[in] _stiffness The joint stop stiffness.
    /// \return The joint stop stiffness.
    /// \sa double Stiffness() const
    public: void SetStiffness(const double _stiffness) const;

    /// \brief Get the joint stop dissipation. The default value is 1.0.
    /// \return The joint stop dissipation.
    /// \sa void SetDissipation(const double _dissipation) const
    public: double Dissipation() const;

    /// \brief Set the joint stop dissipation.
    /// \param[in] _dissipation The joint stop dissipation.
    /// \sa double Dissipation() const
    public: void SetDissipation(const double _dissipation) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer
    private: JointAxisPrivate *dataPtr;
  };
}
#endif
