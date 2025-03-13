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
#include <optional>
#include <gz/math/Vector3.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/Exception.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  struct PoseRelativeToGraph;
  template <typename T> class ScopedGraph;

  /// \brief Helper class to hold contents of a joint axis mimic tag, which
  /// define a Mimic constraint. A Mimic constraint encodes a linear
  /// relationship between the position of two joint axes. One joint axis is
  /// labelled as the *leader* and the other as the *follower*.
  /// The multiplier, offset, and reference parameters determine the linear
  /// relationship according to the following equation:
  ///
  /// follower_position = multiplier * (leader_position - reference) + offset
  class SDFORMAT_VISIBLE MimicConstraint
  {
    /// \brief Constructor with arguments.
    /// \param[in] _joint Name of Joint containing the leader axis.
    /// \param[in] _axis Name of the leader axis, either "axis" or "axis2".
    /// \param[in] _multiplier Multiplier parameter, which represents the ratio
    /// between changes in the follower position relative to changes in the
    /// leader position.
    /// \param[in] _offset Offset to the follower position in the linear
    /// constraint.
    /// \param[in] _reference Reference for the leader position before applying
    /// the multiplier in the linear constraint.
    public: MimicConstraint(
              const std::string &_joint = "",
              const std::string &_axis = "axis",
              double _multiplier = 0.0,
              double _offset = 0.0,
              double _reference = 0.0);

    /// \brief Set the leader joint name.
    /// \param[in] _joint Name of the the joint containing the leader axis.
    public: void SetJoint(const std::string &_joint);

    /// \brief Set the leader axis name, either "axis" or "axis2".
    /// \param[in] _axis Name of the leader axis.
    public: void SetAxis(const std::string &_axis);

    /// \brief Set the multiplier parameter, which represents the ratio
    /// between changes in the follower position relative to changes in the
    /// leader position.
    /// \param[in] _multiplier Multiplier for the Mimic constraint.
    public: void SetMultiplier(double _multiplier);

    /// \brief Set the offset to the follower position in the linear constraint.
    /// \param[in] _offset Offset for the Mimic constraint.
    public: void SetOffset(double _offset);

    /// \brief Set the reference for the leader position before applying
    /// the multiplier in the linear constraint.
    /// \param[in] _reference Reference for the Mimic constraint.
    public: void SetReference(double _reference);

    /// \brief Retrieve the name of the leader joint.
    /// \return Name of the joint containing the leader axis.
    public: const std::string &Joint() const;

    /// \brief Retrieve the name of the leader axis, either "axis" or "axis2".
    /// \return Name of the leader axis.
    public: const std::string &Axis() const;

    /// \brief Retrieve the multiplier parameter, which represents the ratio
    /// between changes in the follower position relative to changes in the
    /// leader position.
    /// \return Multiplier for the Mimic constraint.
    public: double Multiplier() const;

    /// \brief Retrieve the offset to the follower position in the linear
    /// constraint.
    /// \return Offset for the Mimic constraint.
    public: double Offset() const;

    /// \brief Retrieve the reference for the leader position before applying
    /// the multiplier in the linear constraint.
    /// \param[in] _reference Reference for the Mimic constraint.
    public: double Reference() const;

    /// \brief The implementation pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Parameters related to the axis of rotation for rotational joints,
  /// and the axis of translation for prismatic joints.
  class SDFORMAT_VISIBLE JointAxis
  {
    /// \brief Default constructor
    public: JointAxis();

    /// \brief Load the joint axis based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the x,y,z components of the axis unit vector.
    /// The axis is expressed in the frame named in XyzExpressedIn() and
    /// defaults to the joint frame if that method returns an empty string.
    /// The vector should be normalized.
    /// The default value is gz::math::Vector3d::UnitZ which equals
    /// (0, 0, 1).
    /// \return The x,y,z components of the axis unit vector.
    /// \sa void SetXyz(const gz::math::Vector3d &_xyz)
    public: gz::math::Vector3d Xyz() const;

    /// \brief Set the x,y,z components of the axis unit vector.
    /// \param[in] _xyz The x,y,z components of the axis unit vector.
    /// \sa gz::math::Vector3d Xyz() const
    /// \return Errors will have an entry if the norm of the xyz vector is 0.
    public: [[nodiscard]] sdf::Errors SetXyz(
                const gz::math::Vector3d &_xyz);

    /// \brief Set the Mimic constraint parameters.
    /// \param[in] _mimic A MimicConstraint object containing the leader joint
    /// and axis names names, multiplier, offset, and reference to be used
    /// for mimicking.
    /// \sa MimicConstraint Mimic()
    public: void SetMimic(const MimicConstraint &_mimic);

    /// \brief Get the Mimic constraint parameters.
    /// \return A MimicConstraint object containing the leader joint
    /// and axis names names, multiplier, offset, and reference to be used
    /// for mimicking.
    /// \sa void SetMimic(const MimicConstraint)
    public: std::optional<MimicConstraint> Mimic() const;

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
    public: void SetUpper(const double _upper);

    /// \brief Get the value for enforcing the maximum absolute joint effort
    /// that can be applied.
    /// The limit is not enforced if the value is infinity.
    /// The default value is infinity.
    /// \return Symmetric effort limit.
    /// \sa void SetEffort(double _effort)
    public: double Effort() const;

    /// \brief Set the value for enforcing the maximum absolute joint effort
    /// that can be applied.
    /// The limit is not enforced if the value is infinity.
    /// \param[in] _effort Symmetric effort limit.
    /// \sa double Effort() const
    public: void SetEffort(double _effort);

    /// \brief Get the value for enforcing the maximum absolute joint velocity.
    /// The default value is infinity.
    /// \return The value for enforcing the maximum absolute joint velocity.
    /// \sa void SetVelocity(const double _velocity) const
    public: double MaxVelocity() const;

    /// \brief Set the value for enforcing the maximum absolute joint velocity.
    /// \param[in] _velocity The value for enforcing the maximum absolute
    /// joint velocity.
    /// \sa double MaxVelocity() const
    public: void SetMaxVelocity(const double _velocity);

    /// \brief Get the joint stop stiffness. The default value is 1e8.
    /// \return The joint stop stiffness.
    /// \sa void SetStiffness(const double _stiffness) const
    public: double Stiffness() const;

    /// \brief Get the joint stop stiffness.
    /// \param[in] _stiffness The joint stop stiffness.
    /// \return The joint stop stiffness.
    /// \sa double Stiffness() const
    public: void SetStiffness(const double _stiffness);

    /// \brief Get the joint stop dissipation. The default value is 1.0.
    /// \return The joint stop dissipation.
    /// \sa void SetDissipation(const double _dissipation) const
    public: double Dissipation() const;

    /// \brief Set the joint stop dissipation.
    /// \param[in] _dissipation The joint stop dissipation.
    /// \sa double Dissipation() const
    public: void SetDissipation(const double _dissipation);

    /// Get the name of the coordinate frame in which this joint axis's
    /// unit vector is expressed. An empty value implies the parent (joint)
    /// frame.
    /// \return The name of the xyz expressed-in frame.
    public: const std::string& XyzExpressedIn() const;

    /// Set the name of the coordinate frame in which this joint axis's
    /// unit vector is expressed. An empty value implies the parent (joint)
    /// frame.
    /// \param[in] The name of the xyz expressed-in frame.
    public: void SetXyzExpressedIn(const std::string &_frame);

    /// \brief Express xyz unit vector of this axis in the coordinates of
    /// another named frame.
    /// \param[out] _xyz Resolved unit vector.
    /// \param[in] _resolveTo Name of frame in whose coordinates this object
    /// should be resolved. If unset, it is resolved in the coordinates of its
    /// xml parent object, which is always a joint frame.
    /// \return Errors.
    public: Errors ResolveXyz(
        gz::math::Vector3d &_xyz,
        const std::string &_resolveTo = "") const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// joint axis.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[in] _index Index of this joint axis
    /// \return SDF element pointer with updated joint values.
    public: sdf::ElementPtr ToElement(unsigned int _index = 0u) const;

    /// \brief Create and return an SDF element filled with data from this
    /// joint axis.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _index Index of this joint axis
    /// \return SDF element pointer with updated joint values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors,
                                      unsigned int _index = 0u) const;

    /// \brief Give the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Give the scoped PoseRelativeToGraph to be used for resolving
    /// poses. This is private and is intended to be called by
    /// Joint::SetPoseRelativeToGraph.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Allow Joint::SetPoseRelativeToGraph to propagate.
    friend class Joint;

    /// \brief Private data pointer
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
