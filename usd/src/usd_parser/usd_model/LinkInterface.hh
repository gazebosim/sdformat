/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef USD_PARSER_USD_MODEL_LINK_INTERFACE_HH
#define USD_PARSER_USD_MODEL_LINK_INTERFACE_HH

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Inertial.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/Joint.hh"
#include "sdf/Visual.hh"
#include "sdf/Light.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {

  class LinkInterface
  {
  public:
    LinkInterface()
    {
      this->Clear();
    };

    std::string name;

    /// \brief Inertial element
    std::shared_ptr<ignition::math::Inertiald> inertial;

    /// \brief Visual element
    std::shared_ptr<sdf::Visual> visual;

    /// \brief collision element
    std::shared_ptr<sdf::Collision> collision;

    /// \brief If more than one collision element is specified, all collision elements
    /// are placed in this array (the collision member points to the first
    /// element of the array)
    std::vector<std::shared_ptr<sdf::Collision>> collisionArray;

    /// \brief If more than one visual element is specified, all visual elements are
    /// placed in this array (the visual member points to the first element of
    /// the array)
    std::vector<std::shared_ptr<sdf::Visual>> visualArray;

    std::map<std::string, std::shared_ptr<sdf::Light>> lights;

    std::map<std::string, std::shared_ptr<sdf::Sensor>> sensors;

    /// Parent Joint element
    ///  - explicitly stating "parent" because we want directional-ness for
    ///    tree structure
    ///  - every link can have one parent
    std::shared_ptr<sdf::Joint> parentJoint;

    std::vector<std::shared_ptr<sdf::Joint>> childJoints;

    std::vector<std::shared_ptr<LinkInterface>> childLinks;

    std::shared_ptr<LinkInterface> Parent() const
    {
      return parentLink.lock();
    }

    void SetParent(const std::shared_ptr<LinkInterface> &parent)
    {
      parentLink = parent;
    }

    /// \brief Link pose
    ignition::math::Pose3d pose;

    /// \brief Link scale
    ignition::math::Vector3d scale{1, 1, 1};

    void Clear()
    {
      this->name.clear();
      this->collision.reset();
      this->childLinks.clear();
      this->collisionArray.clear();
      this->visualArray.clear();
      this->scale.Set();
      this->pose.Reset();
      scale = ignition::math::Vector3d(1, 1, 1);
    }

  private:
    std::weak_ptr<LinkInterface> parentLink;
  };
  }
  }
}

#endif
