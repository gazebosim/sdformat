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

#ifndef USD_MODEL_LINK_HH
#define USD_MODEL_LINK_HH

#include <string>
#include <vector>
#include <map>

#include "ignition/math/Color.hh"
#include "ignition/math/Inertial.hh"
#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"

#include "joint.hh"

#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/Visual.hh"

#include "types.hh"

namespace usd{

class Link
{
public:
  Link() { this->clear(); };

  std::string name;

  /// inertial element
  std::shared_ptr<ignition::math::Inertiald> inertial;

  /// visual element
  std::shared_ptr<sdf::Visual> visual;

  /// collision element
  std::shared_ptr<sdf::Collision> collision;

  /// if more than one collision element is specified, all collision elements are placed in this array (the collision member points to the first element of the array)
  std::vector<std::shared_ptr<sdf::Collision>> collision_array;

  /// if more than one visual element is specified, all visual elements are placed in this array (the visual member points to the first element of the array)
  std::vector<std::shared_ptr<sdf::Visual>> visual_array;

  /// Parent Joint element
  ///   explicitly stating "parent" because we want directional-ness for tree structure
  ///   every link can have one parent
  JointSharedPtr parent_joint;

  std::vector<JointSharedPtr> child_joints;
  std::vector<LinkSharedPtr> child_links;

  LinkSharedPtr getParent() const
  {return parent_link_.lock();};

  void setParent(const LinkSharedPtr &parent)
  { parent_link_ = parent; }

  ignition::math::Pose3d pose;
  ignition::math::Vector3d scale;

  void clear()
  {
    this->name.clear();
    this->collision.reset();
    this->parent_joint.reset();
    this->child_joints.clear();
    this->child_links.clear();
    this->collision_array.clear();
    this->visual_array.clear();
    this->scale.Set();
    this->pose.Reset();
  };

private:
  LinkWeakPtr parent_link_;

};
}

#endif
