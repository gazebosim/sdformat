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

#include <string>
#include <gz/math/Pose3.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
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
  class Sensor;

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

    /// \brief Get the name of this joint's parent frame.
    /// \return The name of the parent frame.
    public: const std::string &ParentName() const;

    /// \brief Set the name of the parent frame.
    /// \param[in] _name Name of the parent frame.
    public: void SetParentName(const std::string &_name);

    /// \brief Get the name of this joint's child frame.
    /// \return The name of the child frame.
    public: const std::string &ChildName() const;

    /// \brief Set the name of the child frame.
    /// \param[in] _name Name of the child frame.
    public: void SetChildName(const std::string &_name);

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
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the joint.
    /// \sa const gz::math::Pose3d &RawPose() const;
    /// \param[in] _pose The pose of the joint.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

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

    /// \brief Get the displacement along the joint axis for each complete
    /// revolution around the joint axis (only valid for screw joints).
    /// \return The thread pitch with units of meters per revolution with a
    /// positive value corresponding to a right-handed thread.
    public: double ScrewThreadPitch() const;

    /// \brief Set the thread pitch (only valid for screw joints).
    /// \param[in] _threadPitch The thread pitch with units of meters per
    /// revolution with a positive value corresponding to a right-handed thread.
    public: void SetScrewThreadPitch(double _threadPitch);

    /// \brief Get the thread pitch in gazebo-classic format (only valid for
    /// screw joints). This will be deprecated in a future version.
    /// \return The thread pitch with units of radians / meters and a positive
    /// value corresponding to a left-handed thread.
    public: double ThreadPitch() const;

    /// \brief Set the thread pitch in gazebo-classic format (only valid for
    /// screw joints). This will be deprecated in a future version.
    /// \param[in] _threadPitch The thread pitch with units of radians / meters
    /// and a positive value corresponding to a left-handed thread.
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

    /// \brief Get the number of sensors.
    /// \return Number of sensors contained in this Joint object.
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

    /// \brief Create and return an SDF element filled with data from this
    /// joint.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated joint values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Add a sensors to the joint.
    /// \param[in] _sensor Sensor to add.
    /// \return True if successful, false if a sensor with the name already
    /// exists.
    public: bool AddSensor(const Sensor &_sensor);

    /// \brief Remove all sensors.
    public: void ClearSensors();

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

    /// \brief Allow World::Load to call SetPoseRelativeToGraph.
    friend class World;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
