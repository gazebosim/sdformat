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

#ifndef SDF_USD_USD_PARSER_USD_MODEL_MODEL_INTERFACE_HH_
#define SDF_USD_USD_PARSER_USD_MODEL_MODEL_INTERFACE_HH_

#include <string>

#include <ignition/math/Pose3.hh>

#include <sdf/sdf_config.h>

#include "LinkInterface.hh"
#include "sdf/usd/UsdError.hh"

#include "sdf/Joint.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    class ModelInterface
    {
      public:
        const std::string& Name() const
        {
          return name;
        }

        std::shared_ptr<sdf::usd::LinkInterface> Root() const
        {
          return this->rootLink;
        }

        void Clear()
        {
          name.clear();
        }

        void Link(
          const std::string &_name,
          std::shared_ptr<LinkInterface> &_link) const
        {
          std::shared_ptr<LinkInterface> ptr;
          if (this->links.find(_name) == this->links.end())
            ptr.reset();
          else
            ptr = this->links.find(_name)->second;
          _link = ptr;
        }

        UsdErrors InitTree(std::map<std::string, std::string> &parent_link_tree)
        {
          UsdErrors errors;
          // loop through all joints, for every link, assign children links and
          // children joints
          for (
            std::map<std::string, std::shared_ptr<sdf::Joint>>::iterator joint =
              this->joints.begin(); joint != this->joints.end(); joint++)
          {
            std::string parent_link_name = joint->second->ParentLinkName();
            std::string child_link_name = joint->second->ChildLinkName();

            if (parent_link_name.empty() || child_link_name.empty())
            {
              errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
                "Joint [" + joint->second->Name() + "] is missing a parent"
                "and/or child link specification."));
              return errors;
            }
            else
            {
              // find child and parent links
              std::shared_ptr<LinkInterface> child_link, parent_link;
              this->Link(child_link_name, child_link);
              if (!child_link)
              {
                errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
                  "Child link [" + child_link_name + "] of joint [" +
                  joint->first + "] not found"));
                return errors;
              }
              this->Link(parent_link_name, parent_link);
              if (!parent_link)
              {
                errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
                  "Parent link [" + parent_link_name + "] of joint [" +
                  joint->first + "] not found. This is not valid according to "
                  "the SDF spec. Every link you refer to from a joint needs to "
                  "be explicitly defined in the robot description. To fix this "
                  "problem you can either remove this joint [" + joint->first +
                  "] from your USD file, or add \"<link name=\"" +
                  parent_link_name + "\" />\" to your usd file."));
                return errors;
              }

              // set parent link for child link
              child_link->SetParent(parent_link);

              // set parent joint for child link
              child_link->parentJoint = joint->second;

              // set child joint for parent link
              parent_link->childJoints.push_back(joint->second);

              // set child link for parent link
              parent_link->childLinks.push_back(child_link);

              // fill in child/parent string map
              parent_link_tree[child_link->name] = parent_link_name;
            }
          }
          return errors;
        }

        UsdErrors InitRoot(std::map<std::string, std::string> &parent_link_tree)
        {
          UsdErrors errors;
          this->rootLink.reset();

          // find the links that have no parent in the tree
          std::map<
            std::string,
            std::shared_ptr<LinkInterface>>::const_iterator linkIterator;
          for (linkIterator = this->links.begin();
               linkIterator != this->links.end(); ++linkIterator)
          {
            std::map<std::string, std::string >::const_iterator parent =
              parent_link_tree.find(linkIterator->first);
            if (parent == parent_link_tree.end())
            {
              // store root link
              if (!this->rootLink)
              {
                this->Link(linkIterator->first, this->rootLink);
              }
              // we already found a root link
              else
              {
                errors.push_back(
                  UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
                    "Two root links found: [" + this->rootLink->name +
                    "] and [" + linkIterator->first + "]"));
                return errors;
              }
            }
          }
          if (!this->rootLink)
          {
            errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
              "No root link found. The robot xml is not a valid tree."));
            return errors;
          }
          return errors;
        }

        /// \brief The root is always a link (the parent of the tree describing
        /// the robot)
        std::shared_ptr<sdf::usd::LinkInterface> rootLink;

        /// \brief complete list of Joints
        std::map<std::string, std::shared_ptr<sdf::Joint>> joints;

        /// \brief complete list of Links
        std::map<std::string, std::shared_ptr<sdf::usd::LinkInterface>> links;

        /// \brief Model pose
        ignition::math::Pose3d pose;

        /// \brief The name of the robot model
        std::string name;
    };
  }
  }
}

#endif
