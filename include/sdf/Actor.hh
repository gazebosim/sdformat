/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_ACTOR_HH_
#define SDF_ACTOR_HH_

#include <memory>
#include <string>

#include <gz/math/Pose3.hh>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/Link.hh"
#include "sdf/Joint.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class AnimationPrivate;

  // Forward declare private data class.
  class WaypointPrivate;

  // Forward declare private data class.
  class TrajectoryPrivate;

  // Forward declare private data class.
  class ActorPrivate;

  /// \brief Animation in Actor.
  class SDFORMAT_VISIBLE Animation
  {
    /// \brief Default constructor
    public: Animation();

    /// \brief Copy constructor
    /// \param[in] _animation Animation to copy.
    public: Animation(const Animation &_animation);

    /// \brief Move constructor
    /// \param[in] _animation Animation to move.
    public: Animation(Animation &&_animation) noexcept;

    /// \brief Destructor
    public: ~Animation();

    /// \brief Move assignment operator.
    /// \param[in] _animation Animation to move.
    /// \return Reference to this.
    public: Animation &operator=(Animation &&_animation);

    /// \brief Assignment operator.
    /// \param[in] _animation The animation to set values from.
    /// \return *this
    public: Animation &operator=(const Animation &_animation);

    /// \brief Load the animation based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the animation.
    /// \return Name of the animation.
    public: const std::string &Name() const;

    /// \brief Set the name of the animation.
    /// \param[in] _name Name of the animation.
    public: void SetName(const std::string &_name);

    /// \brief Get the animation filename.
    /// \return Filename of the animation.
    public: const std::string &Filename() const;

    /// \brief Set the filename of the animation.
    /// \param[in] _filename Path to animation file.
    public: void SetFilename(const std::string &_filename);

    /// \brief The path to the file where this element was loaded from.
    /// \return Full path to the file on disk.
    public: const std::string &FilePath() const;

    /// \brief Set the path to the file where this element was loaded from.
    /// \paramp[in] _filePath Full path to the file on disk.
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Get the scale for the animation skeleton.
    /// \return Scale of the animation skeleton.
    public: double Scale() const;

    /// \brief Set the scale of the animation skeleton.
    /// \param[in] _scale Scale for animation skeleton.
    public: void SetScale(double _scale);

    /// \brief Get whether the animation is interpolated on X.
    /// \return True if interpolated on X.
    public: bool InterpolateX() const;

    /// \brief Set whether the animation is interpolated on X.
    /// \param[in] _interpolateX True to indicate interpolation on X.
    public: void SetInterpolateX(bool _interpolateX);

    /// \brief Copy animation from an Animation instance.
    /// \param[in] _animation The animation to set values from.
    public: void CopyFrom(const Animation &_animation);

    /// \brief Private data pointer.
    private: AnimationPrivate *dataPtr = nullptr;
  };

  /// \brief Waypoint for Trajectory.
  class SDFORMAT_VISIBLE Waypoint
  {
    /// \brief Default constructor
    public: Waypoint();

    /// \brief Copy constructor
    /// \param[in] _waypoint Waypoint to copy.
    public: Waypoint(const Waypoint &_waypoint);

    /// \brief Move constructor
    /// \param[in] _waypoint Waypoint to move.
    public: Waypoint(Waypoint &&_waypoint) noexcept;

    /// \brief Destructor
    public: ~Waypoint();

    /// \brief Move assignment operator.
    /// \param[in] _waypoint Waypoint to move.
    /// \return Reference to this.
    public: Waypoint &operator=(Waypoint &&_waypoint);

    /// \brief Assignment operator.
    /// \param[in] _waypoint The waypoint to set values from.
    /// \return *this
    public: Waypoint &operator=(const Waypoint &_waypoint);

    /// \brief Load the waypoint based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the time in seconds when the pose should be reached.
    /// \return Time in seconds.
    public: double Time() const;

    /// \brief Set the time in seconds when the pose should be reached.
    /// \param[in] _time Time in seconds for the pose to be reached.
    public: void SetTime(double _time);

    /// \brief Get the pose to be reached.
    /// \return Pose to be reached.
    public: gz::math::Pose3d Pose() const;

    /// \brief Set the pose to be reached.
    /// \param[in] _pose Pose to be reached.
    public: void SetPose(const gz::math::Pose3d &_pose);

    /// \brief Copy waypoint from an Waypoint instance.
    /// \param[in] _waypoint The waypoint to set values from.
    public: void CopyFrom(const Waypoint &_waypoint);

    /// \brief Private data pointer.
    private: WaypointPrivate *dataPtr = nullptr;
  };

  /// \brief Trajectory for Animation.
  class SDFORMAT_VISIBLE Trajectory
  {
    /// \brief Default constructor
    public: Trajectory();

    /// \brief Copy constructor
    /// \param[in] _trajectory Trajectory to copy.
    public: Trajectory(const Trajectory &_trajectory);

    /// \brief Move constructor
    /// \param[in] _trajectory Trajectory to move.
    public: Trajectory(Trajectory &&_trajectory) noexcept;

    /// \brief Destructor
    public: ~Trajectory();

    /// \brief Move assignment operator.
    /// \param[in] _trajectory Trajectory to move.
    /// \return Reference to this.
    public: Trajectory &operator=(Trajectory &&_trajectory);

    /// \brief Assignment operator.
    /// \param[in] _trajectory The trajectory to set values from.
    /// \return *this
    public: Trajectory &operator=(const Trajectory &_trajectory);

    /// \brief Load the trajectory based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the unique id of the trajectory.
    /// \return Trajectory id.
    public: uint64_t Id() const;

    /// \brief Set the ID of the trajectory.
    /// \param[in] _id Trajectory Id.
    public: void SetId(uint64_t _id);

    /// \brief Get the type of the trajectory.
    /// \return Type of the trajectory.
    public: const std::string &Type() const;

    /// \brief Set the animation type of the trajectory.
    /// (should match the animation name).
    /// \param[in] _type Animation type to be played.
    public: void SetType(const std::string &_type);

    /// \brief Get the tension of the trajectory spline.
    /// \return Tension of the trajectory spline.
    public: double Tension() const;

    /// \brief Set the tension of trajectory spline.
    /// \param[in] _tension Tension for the trajectory spline.
    public: void SetTension(double _tension);

    /// \brief Get the number of waypoints.
    /// \return Number of waypoints.
    public: uint64_t WaypointCount() const;

    /// \brief Get a waypoint based on an index.
    /// \param[in] _index Index of the waypoint. The index should be in the
    /// range [0..WaypointCount()).
    /// \return Pointer to the waypoint. Nullptr if the index does not exist.
    /// \sa uint64_t WaypointCount() const
    public: const Waypoint *WaypointByIndex(uint64_t _index) const;

    /// \brief Add a new waypoint.
    /// \param[in] _waypoint Waypoint to be added.
    public: void AddWaypoint(const Waypoint &_waypoint);

    /// \brief Copy trajectory from a trajectory instance.
    /// \param[in] _trajectory The trajectory to set values from.
    public: void CopyFrom(const Trajectory &_trajectory);

    /// \brief Private data pointer.
    private: TrajectoryPrivate *dataPtr = nullptr;
  };


  /// \brief Provides a description of an actor.
  class SDFORMAT_VISIBLE Actor
  {
    /// \brief Default constructor
    public: Actor();

    /// \brief Copy constructor
    /// \param[in] _actor Actor to copy.
    public: Actor(const Actor &_actor);

    /// \brief Move constructor
    /// \param[in] _actor Actor to move.
    public: Actor(Actor &&_actor) noexcept;

    /// \brief Destructor
    public: ~Actor();

    /// \brief Move assignment operator.
    /// \param[in] _actor Actor to move.
    /// \return Reference to this.
    public: Actor &operator=(Actor &&_actor);

    /// \brief Assignment operator.
    /// \param[in] _actor The actor to set values from.
    /// \return *this
    public: Actor &operator=(const Actor &_actor);

    /// \brief Copy dataPtr from an actor instance.
    /// \param[in] _actor The actor to set values from.
    public: void CopyFrom(const Actor &_actor);

    /// \brief Load the actor based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the actor.
    /// \return Name of the actor.
    public: std::string &Name() const;

    /// \brief Set the name of the actor.
    /// \param[in] _name Name of the actor.
    public: void SetName(const std::string &_name);

    /// \brief Get the pose of the actor. This is the pose of the actor
    /// as specified in SDF (<actor> <pose> ... </pose></actor>), and is
    /// typically used to express the position and rotation of an actor in a
    /// global coordinate frame.
    /// \return The pose of the actor.
    /// \deprecated See RawPose.
    public: const gz::math::Pose3d &Pose() const
        SDF_DEPRECATED(9.0);

    /// \brief Set the pose of the actor.
    /// \sa const gz::math::Pose3d &Pose() const
    /// \param[in] _pose The new actor pose.
    /// \deprecated See SetRawPose.
    public: void SetPose(const gz::math::Pose3d &_pose)
        SDF_DEPRECATED(9.0);

    /// \brief Get the pose of the actor. This is the pose of the actor
    /// as specified in SDF (<actor> <pose> ... </pose></actor>), and is
    /// typically used to express the position and rotation of an actor in a
    /// global coordinate frame.
    /// \return The pose of the actor.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the actor.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The new actor pose.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the world frame.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the world frame.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get the name of the coordinate frame in which this actor's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \return The name of the pose frame.
    /// \deprecated See PoseRelativeTo.
    public: const std::string &PoseFrame() const
        SDF_DEPRECATED(9.0);

    /// \brief Set the name of the coordinate frame in which this actor's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \param[in] _frame The name of the pose frame.
    /// \deprecated See SetPoseRelativeTo.
    public: void SetPoseFrame(const std::string &_frame)
        SDF_DEPRECATED(9.0);

    /// \brief The path to the file where this element was loaded from.
    /// \return Full path to the file on disk.
    public: const std::string &FilePath() const;

    /// \brief Set the path to the file where this element was loaded from.
    /// \paramp[in] _filePath Full path to the file on disk.
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Get the skin filename.
    /// \return Constant skin filename.
    public: const std::string &SkinFilename() const;

    /// \brief Set the skin filename.
    /// \param[in] _skinFilename Skin filename.
    public: void SetSkinFilename(std::string _skinFilename);

    /// \brief Get the skin scale.
    /// \return Constant skin filename.
    public: double SkinScale() const;

    /// \brief Set the skin scale.
    /// \param[in] _skinScale Skin scale.
    public: void SetSkinScale(double _skinScale);

    /// \brief Get the number of animations.
    /// \return Number of animations.
    public: uint64_t AnimationCount() const;

    /// \brief Get an animation based on an index.
    /// \param[in] _index Index of the animation. The index should be in the
    /// range [0..AnimationCount()).
    /// \return Pointer to the animation. Nullptr if the index does not exist.
    /// \sa uint64_t AnimationCount() const
    public: const Animation *AnimationByIndex(uint64_t _index) const;

    /// \brief Get whether an animation name exists.
    /// \param[in] _name Name of the animation to check.
    /// \return True if there exists an animation with the given name.
    public: bool AnimationNameExists(const std::string &_name) const;

    /// \brief Add a new animation.
    /// \param[in] _anim Animation to be added.
    public: void AddAnimation(const Animation &_anim);

    /// \brief Get whether the animation plays in loop.
    /// \return True if the animation plays in loop.
    public: bool ScriptLoop() const;

    /// \brief Set whether the animation plays in loop.
    /// \param[in] _scriptLoop True to indicate that the animation
    /// plays in loop.
    public: void SetScriptLoop(bool _scriptLoop);

    /// \brief Get the time (in seconds) of delay to start.
    /// \return Time of delay to start.
    public: double ScriptDelayStart() const;

    /// \brief Set the delay time to start.
    /// \param[in] _scriptDelayStart Time of delay to start.
    public: void SetScriptDelayStart(double _scriptDelayStart);

    /// \brief Get whether the animation plays when simulation starts.
    /// \return True if the animation plays when simulation starts.
    public: bool ScriptAutoStart() const;

    /// \brief Set whether the animation plays when simulation starts.
    /// \param[in] _staticAutoStart True to indicate that the animation
    /// plays when simulation starts.
    public: void SetScriptAutoStart(bool _scriptAutoStart);

    /// \brief Get the number of trajectories.
    /// \return Number of trajectories.
    public: uint64_t TrajectoryCount() const;

    /// \brief Get a trajectory based on an index.
    /// \param[in] _index Index of the trajectory. The index should be in the
    /// range [0..TrajectoryCount()).
    /// \return Pointer to the trajectory. Nullptr if the index does not exist.
    /// \sa uint64_t TrajectoryCount() const
    public: const Trajectory *TrajectoryByIndex(uint64_t _index) const;

    /// \brief Get whether a trajectory id exists.
    /// \param[in] _id Id of the trajectory to check.
    /// \return True if there exists a trajectory with the given name.
    public: bool TrajectoryIdExists(uint64_t _id) const;

    /// \brief Add a new trajectory.
    /// \param[in] _traj Trajectory to be added.
    public: void AddTrajectory(const Trajectory &_traj);

    /// \brief Get the number of links.
    /// \return Number of links.
    public: uint64_t LinkCount() const;

    /// \brief Get a link based on an index.
    /// \param[in] _index Index of the link. The index should be in the
    /// range [0..LinkCount()).
    /// \return Pointer to the link. Nullptr if the index does not exist.
    /// \sa uint64_t LinkCount() const
    public: const Link *LinkByIndex(uint64_t _index) const;

    /// \brief Get whether a link name exists.
    /// \param[in] _name Name of the link to check.
    /// \return True if there exists a link with the given name.
    public: bool LinkNameExists(const std::string &_name) const;

    /// \brief Get the number of joints.
    /// \return Number of joints.
    public: uint64_t JointCount() const;

    /// \brief Get a joint based on an index.
    /// \param[in] _index Index of the joint. The index should be in the
    /// range [0..JointCount()).
    /// \return Pointer to the joint. Nullptr if the index does not exist.
    /// \sa uint64_t JointCount() const
    public: const Joint *JointByIndex(uint64_t _index) const;

    /// \brief Get whether a joint name exists.
    /// \param[in] _name Name of the joint to check.
    /// \return True if there exists a joint with the given name.
    public: bool JointNameExists(const std::string &_name) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: ActorPrivate *dataPtr = nullptr;
  };
  }
}
#endif
