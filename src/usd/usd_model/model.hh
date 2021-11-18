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

#ifndef USD_MODEL_MODEL_HH
#define USD_MODEL_MODEL_HH

#include <iostream>

#include <usd_model/link.hh>
#include <usd_model/types.hh>

#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Light.hh>

namespace usd {

class ModelInterface
{
public:
  LinkConstSharedPtr getRoot(void) const{return this->root_link_;};
  LinkConstSharedPtr getLink(const std::string& name) const
  {
    LinkConstSharedPtr ptr;
    if (this->links_.find(name) == this->links_.end())
      ptr.reset();
    else
      ptr = this->links_.find(name)->second;
    return ptr;
  };

  std::shared_ptr<sdf::Joint> getJoint(const std::string& name) const
  {
    std::shared_ptr<sdf::Joint> ptr;
    if (this->joints_.find(name) == this->joints_.end())
      ptr.reset();
    else
      ptr = this->joints_.find(name)->second;
    return ptr;
  };


  const std::string& getName() const {return name_;};
  void getLinks(std::vector<LinkSharedPtr >& links) const
  {
    for (std::map<std::string,LinkSharedPtr>::const_iterator link = this->links_.begin();link != this->links_.end(); link++)
    {
      links.push_back(link->second);
    }
  };

  void clear()
  {
    name_.clear();
    this->links_.clear();
    this->joints_.clear();
    this->materials_.clear();
    this->root_link_.reset();
  };

  /// non-const getLink()
  void getLink(const std::string& name, LinkSharedPtr &link) const
  {
    LinkSharedPtr ptr;
    if (this->links_.find(name) == this->links_.end())
      ptr.reset();
    else
      ptr = this->links_.find(name)->second;
    link = ptr;
  };

  /// non-const getMaterial()
  std::shared_ptr<sdf::Material> getMaterial(const std::string& name) const
  {
    std::shared_ptr<sdf::Material> ptr;
    if (this->materials_.find(name) == this->materials_.end())
      ptr.reset();
    else
      ptr = this->materials_.find(name)->second;
    return ptr;
  };

  void initTree(std::map<std::string, std::string> &parent_link_tree)
  {
    // loop through all joints, for every link, assign children links and children joints
    for (std::map<std::string, std::shared_ptr<sdf::Joint>>::iterator joint = this->joints_.begin();
         joint != this->joints_.end(); joint++)
    {
      std::string parent_link_name = joint->second->ParentLinkName();
      std::string child_link_name = joint->second->ChildLinkName();

      if (parent_link_name.empty() || child_link_name.empty())
      {
        throw ParseError("Joint [" + joint->second->Name() + "] is missing a parent and/or child link specification.");
      }
      else
      {
        // find child and parent links
        LinkSharedPtr child_link, parent_link;
        this->getLink(child_link_name, child_link);
        if (!child_link)
        {
          throw ParseError("child link [" + child_link_name + "] of joint [" + joint->first + "] not found");
        }
        this->getLink(parent_link_name, parent_link);
        if (!parent_link)
        {
          throw ParseError("parent link [" + parent_link_name + "] of joint [" + joint->first + "] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [" + joint->first + "] from your urdf file, or add \"<link name=\"" + parent_link_name + "\" />\" to your urdf file.");
        }
        std::cerr << "Joint name: " << joint->second->Name()
                  << " child_link " << child_link->name
                  << " parent_link: " << parent_link->name << '\n';

        //set parent link for child link
        child_link->setParent(parent_link);

        //set parent joint for child link
        // std::cerr << "joint->second " << joint->second->parent_link_name << " " << joint->second->child_link_name << '\n';
        child_link->parent_joint = joint->second;

        //set child joint for parent link
        parent_link->child_joints.push_back(joint->second);

        std::cerr << "Added to parant_link " << parent_link->name << " this child: " << child_link->name << '\n';
        //set child link for parent link
        parent_link->child_links.push_back(child_link);

        // fill in child/parent string map
        parent_link_tree[child_link->name] = parent_link_name;
      }
    }
  }

  void initRoot(std::map<std::string, std::string> &parent_link_tree)
  {
    this->root_link_.reset();

    // find the links that have no parent in the tree
    std::map<std::string, LinkSharedPtr>::const_iterator l;
    for (l = this->links_.begin(); l != this->links_.end(); ++l)
    {
      std::map<std::string, std::string >::const_iterator parent =
        parent_link_tree.find(l->first);
      if (parent == parent_link_tree.end())
      {
        // store root link
        if (!this->root_link_)
        {
          getLink(l->first, this->root_link_);
        }
        // we already found a root link
        else
        {
          std::shared_ptr<sdf::Joint> joint = nullptr;
          joint = std::make_shared<sdf::Joint>();
          joint->SetParentLinkName(this->root_link_->name);
          joint->SetChildLinkName(l->first);
          joint->SetName(this->root_link_->name + "_" + l->first + "_joint");
          joint->SetType(sdf::JointType::FIXED);
          this->joints_.insert(make_pair(joint->Name(), joint));
          initTree(parent_link_tree);
          l--;
          // throw ParseError("Two root links found: [" + this->root_link_->name + "] and [" + l->first + "]");
        }
      }
    }
    if (!this->root_link_)
    {
      throw ParseError("No root link found. The robot xml is not a valid tree.");
    }
  }

  /// \brief The root is always a link (the parent of the tree describing the robot)
  LinkSharedPtr root_link_;

  /// \brief complete list of Links
  std::map<std::string, LinkSharedPtr> links_;
  /// \brief complete list of Joints
  std::map<std::string, std::shared_ptr<sdf::Joint>> joints_;
  /// \brief complete list of Materials
  std::map<std::string, std::shared_ptr<sdf::Material>> materials_;

  std::map<std::string, std::shared_ptr<sdf::Light>> lights_;

  /// \brief The name of the robot model
  std::string name_;
};
}

#endif
