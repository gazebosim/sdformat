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
#ifndef SDF_JOINT_HH_
#define SDF_JOINT_HH_

#include <memory>
#include <string>
#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>
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

  // Forward declarations.
  class JointAxis;
  struct FrameAttachedToGraph;
  struct PoseRelativeToGraph;
  template <typename T> class ScopedGraph;

  /// \enum JointType
  /// \brief The set of joint types. INVALID indicates that joint type has
  /// not been set, or has not been set correctly.
  enum class JointType
  {
    /// \brief An invalid joint. This indicates an error since a joint type
    /// is required to fully specify a joint.
    INVALID = 0,

    /// \brief A ball and socket joint
    BALL = 1,

    /// \brief A hinge joint that rotates on a single axis with a
    /// continuous range of motion
    CONTINUOUS = 2,

    /// \brief A joint with zero degrees of freedom that rigidly connects two
    /// links.
    FIXED = 3,

    /// \brief Geared revolute joint
    GEARBOX = 4,

    /// \brief A sliding joint that slides along an axis with a limited range
    /// specified by upper and lower limits
    PRISMATIC = 5,

    /// \brief A hinge joint that rotates on a single axis with a fixed range
    /// of motion
    REVOLUTE = 6,

    /// \brief Same as two revolute joints connected in series
    REVOLUTE2 = 7,

    /// \brief A single degree of freedom joint with coupled sliding and
    /// rotational motion
    SCREW = 8,

    /// \brief Similar to a ball joint, but constrains one degree of freedom
    UNIVERSAL = 9
  };

  class SDFORMAT_VISIBLE Joint
  {
    /// \brief Default constructor
    public: Joint();

    /// \brief Load the joint based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the joint.
    /// The name of the joint must be unique within the scope of a Model.
    /// \return Name of the joint.
    public: const std::string &Name() const;

    /// \brief Set the name of the joint.
    /// The name of the joint must be unique within the scope of a Model.
    /// \param[in] _name Name of the joint.
    public: void SetName(const std::string &_name);

    /// \brief Get the joint type
    /// \return Type of joint.
    public: JointType Type() const;

    /// \brief Set the joint type
    /// \param[in] _jointType The type of joint.
    public: void SetType(const JointType _jointType);

    /// \brief Get the name of this joint's parent link.
    /// \return The name of the parent link.
    public: const std::string &ParentLinkName() const;

    /// \brief Set the name of the parent link.
    /// \param[in] _name Name of the parent link.
    public: void SetParentLinkName(const std::string &_name);

    /// \brief Get the name of this joint's child link.
    /// \return The name of the child link.
    public: const std::string &ChildLinkName() const;

    /// \brief Set the name of the child link.
    /// \param[in] _name Name of the child link.
    public: void SetChildLinkName(const std::string &_name);

    /// \brief Resolve the name of the child link from the
    /// FrameAttachedToGraph.
    /// \param[out] _body Name of child link of this joint.
    /// \return Errors.
    public: Errors ResolveChildLink(std::string &_link) const;

    /// \brief Resolve the name of the parent link from the
    /// FrameAttachedToGraph. It will return the name of a link or "world".
    /// \param[out] _body Name of parent link of this joint.
    /// \return Errors.
    public: Errors ResolveParentLink(std::string &_link) const;

    /// \brief Get a joint axis.
    /// \param[in] _index This value specifies which axis to get. A value of
    /// zero corresponds to the first axis, which is the <axis> SDF
    /// element. Any other value will return the second axis, which is the
    /// <axis2> SDF element.
    /// \return A JointAxis for either the first or second joint axis. A
    /// return value of nullptr indicates that the axis is not
    /// specified.
    public: const JointAxis *Axis(const unsigned int _index = 0) const;

    /// \brief Set a joint axis.
    /// \param[in] _index This value specifies which axis to set. A value of
    /// zero corresponds to the first axis, which is the <axis> SDF
    /// element. Any other value will set the second axis, which is the
    /// <axis2> SDF element.
    /// \param[in] _axis The JointAxis of the joint
    public: void SetAxis(const unsigned int _index, const JointAxis &_axis);

    /// \brief Get the pose of the joint. This is the pose of the joint
    /// as specified in SDF (<joint> <pose> ... </pose></joint>).
    /// Transformations have not been applied to the return value.
    /// \return The pose of the joint. This is the raw pose value, as set in
    /// the SDF file.
    public: const ignition::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the joint.
    /// \sa const ignition::math::Pose3d &RawPose() const;
    /// \param[in] _pose The pose of the joint.
    public: void SetRawPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the child link frame.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the child link frame.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get the thread pitch (only valid for screw joints)
    /// \return The thread pitch
    public: double ThreadPitch() const;

    /// \brief Set the thread pitch (only valid for screw joints)
    /// \param[in] _threadPitch The thread pitch of the joint
    public: void SetThreadPitch(double _threadPitch);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get SemanticPose object of this object to aid in resolving
    /// poses.
    /// \return SemanticPose object for this link.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Give the scoped FrameAttachedToGraph to be used for resolving
    /// parent and child link names. This is private and is intended to be
    /// called by Model::Load.
    /// \param[in] _graph scoped FrameAttachedToGraph object.
    private: void SetFrameAttachedToGraph(
        sdf::ScopedGraph<FrameAttachedToGraph> _graph);

    /// \brief Give the scoped PoseRelativeToGraph to be used for resolving
    /// poses. This is private and is intended to be called by Model::Load.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Allow Model::Load to call SetPoseRelativeToGraph.
    friend class Model;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
