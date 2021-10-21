/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef USD_MODEL_JOINTS_HH
#define USD_MODEL_JOINTS_HH

#include <string>
#include <vector>

#include "usd_model/pose.hh"
#include "usd_model/types.hh"

namespace usd{

class Link;

class JointDynamics
{
public:
  JointDynamics() { this->clear(); };
  double damping;
  double friction;

  void clear()
  {
    damping = 0;
    friction = 0;
  };
};

class JointLimits
{
public:
  JointLimits() { this->clear(); };
  double lower;
  double upper;
  double effort;
  double velocity;

  void clear()
  {
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
  };
};

/// \brief Parameters for Joint Safety Controllers
class JointSafety
{
public:
  /// clear variables on construction
  JointSafety() { this->clear(); };

  ///
  /// IMPORTANT:  The safety controller support is very much PR2 specific, not intended for generic usage.
  ///
  /// Basic safety controller operation is as follows
  ///
  /// current safety controllers will take effect on joints outside the position range below:
  ///
  /// position range: [JointSafety::soft_lower_limit  + JointLimits::velocity / JointSafety::k_position,
  ///                  JointSafety::soft_uppper_limit - JointLimits::velocity / JointSafety::k_position]
  ///
  /// if (joint_position is outside of the position range above)
  ///     velocity_limit_min = -JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_lower_limit)
  ///     velocity_limit_max =  JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_upper_limit)
  /// else
  ///     velocity_limit_min = -JointLimits::velocity
  ///     velocity_limit_max =  JointLimits::velocity
  ///
  /// velocity range: [velocity_limit_min + JointLimits::effort / JointSafety::k_velocity,
  ///                  velocity_limit_max - JointLimits::effort / JointSafety::k_velocity]
  ///
  /// if (joint_velocity is outside of the velocity range above)
  ///     effort_limit_min = -JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_min)
  ///     effort_limit_max =  JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_max)
  /// else
  ///     effort_limit_min = -JointLimits::effort
  ///     effort_limit_max =  JointLimits::effort
  ///
  /// Final effort command sent to the joint is saturated by [effort_limit_min,effort_limit_max]
  ///
  /// Please see wiki for more details: http://www.ros.org/wiki/pr2_controller_manager/safety_limits
  ///
  double soft_upper_limit;
  double soft_lower_limit;
  double k_position;
  double k_velocity;

  void clear()
  {
    soft_upper_limit = 0;
    soft_lower_limit = 0;
    k_position = 0;
    k_velocity = 0;
  };
};


class JointCalibration
{
public:
  JointCalibration() { this->clear(); };
  double reference_position;
  DoubleSharedPtr rising, falling;

  void clear()
  {
    reference_position = 0;
  };
};

class JointMimic
{
public:
  JointMimic() { this->clear(); };
  double offset;
  double multiplier;
  std::string joint_name;

  void clear()
  {
    offset = 0.0;
    multiplier = 0.0;
    joint_name.clear();
  };
};


class Joint
{
public:

  Joint() { this->clear(); };

  std::string name;
  enum
  {
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
  } type;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3 axis;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;
  /// transform from Parent Link frame to Joint frame
  Pose  parent_to_joint_origin_transform;

  /// Joint Dynamics
  JointDynamicsSharedPtr dynamics;

  /// Joint Limits
  JointLimitsSharedPtr limits;

  /// Unsupported Hidden Feature
  JointSafetySharedPtr safety;

  /// Unsupported Hidden Feature
  JointCalibrationSharedPtr calibration;

  /// Option to Mimic another Joint
  JointMimicSharedPtr mimic;

  void clear()
  {
    this->axis.clear();
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->parent_to_joint_origin_transform.clear();
    this->dynamics.reset();
    this->limits.reset();
    this->safety.reset();
    this->calibration.reset();
    this->mimic.reset();
    this->type = UNKNOWN;
  };
};

}

#endif
