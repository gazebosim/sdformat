/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <string.h>

#include "sdf/sdf_config.h"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/FrameSemantics.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/ign.hh"
#include "sdf/parser.hh"
#include "sdf/system_util.hh"

//////////////////////////////////////////////////
/// \brief Check that for each model, the canonical_link attribute value
/// matches the name of a link in the model if the attribute is set and
/// not empty.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all models have valid canonical_link attributes.
bool checkCanonicalLinkNames(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelCanonicalLinkName = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    std::string canonicalLink = _model->CanonicalLinkName();
    if (!canonicalLink.empty() && !_model->LinkNameExists(canonicalLink))
    {
      std::cerr << "Error: canonical_link with name[" << canonicalLink
                << "] not found in model with name[" << _model->Name()
                << "]."
                << std::endl;
      modelResult = false;
    }
    return modelResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelCanonicalLinkName(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelCanonicalLinkName(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief Check that for each frame, the attached_to attribute value
/// does not match its own frame name but does match the name of a
/// link, joint, or other frame in the model if the attribute is set and
/// not empty.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all frames have valid attached_to attributes.
bool checkFrameAttachedToNames(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelFrameAttachedToNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t f = 0; f < _model->FrameCount(); ++f)
    {
      auto frame = _model->FrameByIndex(f);

      const std::string &attachedTo = frame->AttachedTo();

      // the attached_to attribute is always permitted to be empty
      if (attachedTo.empty())
      {
        continue;
      }

      if (attachedTo == frame->Name())
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] is identical to frame name[" << frame->Name()
                  << "], causing a graph cycle "
                  << "in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }
      else if (!_model->LinkNameExists(attachedTo) &&
               !_model->JointNameExists(attachedTo) &&
               !_model->FrameNameExists(attachedTo))
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] specified by frame with name[" << frame->Name()
                  << "] does not match a link, joint, or frame name "
                  << "in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }
    }
    return modelResult;
  };

  auto checkWorldFrameAttachedToNames = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    for (uint64_t f = 0; f < _world->FrameCount(); ++f)
    {
      auto frame = _world->FrameByIndex(f);

      const std::string &attachedTo = frame->AttachedTo();

      // the attached_to attribute is always permitted to be empty
      if (attachedTo.empty())
      {
        continue;
      }

      if (attachedTo == frame->Name())
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] is identical to frame name[" << frame->Name()
                  << "], causing a graph cycle "
                  << "in world with name[" << _world->Name()
                  << "]."
                  << std::endl;
        worldResult = false;
      }
      else if (!_world->ModelNameExists(attachedTo) &&
               !_world->FrameNameExists(attachedTo))
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] specified by frame with name[" << frame->Name()
                  << "] does not match a model or frame name "
                  << "in world with name[" << _world->Name()
                  << "]."
                  << std::endl;
        worldResult = false;
      }
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelFrameAttachedToNames(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    result = checkWorldFrameAttachedToNames(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelFrameAttachedToNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief For each model, check that the kinematic graphs build without
/// errors.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all kinematic graphs are valid.
bool checkKinematicGraph(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelKinematicGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    sdf::KinematicGraph graph;
    auto errors = sdf::buildKinematicGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelKinematicGraph(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelKinematicGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief For the world and each model, check that the attached_to graphs
/// build without errors and have no cycles.
/// Confirm that following directed edges from each vertex in the graph
/// leads to a model, link, or world frame.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all attached_to graphs are valid.
bool checkFrameAttachedToGraph(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelFrameAttachedToGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    sdf::FrameAttachedToGraph graph;
    auto errors = sdf::buildFrameAttachedToGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  auto checkWorldFrameAttachedToGraph = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    sdf::FrameAttachedToGraph graph;
    auto errors = sdf::buildFrameAttachedToGraph(graph, _world);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      worldResult = false;
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelFrameAttachedToGraph(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    result = checkWorldFrameAttachedToGraph(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelFrameAttachedToGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief For the world and each model, check that the attached_to graphs
/// build without errors and have no cycles.
/// Confirm that following directed edges from each vertex in the graph
/// leads to a model, link, or world frame.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all attached_to graphs are valid.
bool checkPoseRelativeToGraph(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelPoseRelativeToGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    sdf::PoseRelativeToGraph graph;
    auto errors = sdf::buildPoseRelativeToGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  auto checkWorldPoseRelativeToGraph = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    sdf::PoseRelativeToGraph graph;
    auto errors = sdf::buildPoseRelativeToGraph(graph, _world);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      worldResult = false;
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelPoseRelativeToGraph(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    result = checkWorldPoseRelativeToGraph(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelPoseRelativeToGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief Helper function for checking validity of pose relative_to attribute
/// values in the model scope.
/// \param[in] _model Model object that contains object _t in its scope.
/// \param[in] _t Object whose pose relative_to attribute is to be checked
/// (may be Link, Joint, Frame, Collision, etc.).
/// \param[in] _typeName Name of type T for use in console messages.
/// \return True if relative_to attribute values a valid.
template <typename T>
bool checkModelPoseRelativeTo(
    const sdf::Model *_model,
    const T *_t,
    const std::string &_typeName)
{
  const std::string &relativeTo = _t->PoseRelativeTo();

  // the relative_to attribute is always permitted to be empty
  if (relativeTo.empty())
  {
    return true;
  }

  if (relativeTo == _t->Name())
  {
    std::cerr << "Error: relative_to name[" << relativeTo
              << "] is identical to " << _typeName << " name[" << _t->Name()
              << "], causing a graph cycle "
              << "in model with name[" << _model->Name()
              << "]."
              << std::endl;
    return false;
  }
  else if (!_model->LinkNameExists(relativeTo) &&
           !_model->JointNameExists(relativeTo) &&
           !_model->FrameNameExists(relativeTo))
  {
    std::cerr << "Error: relative_to name[" << relativeTo
              << "] specified by " << _typeName << " with name[" << _t->Name()
              << "] does not match a link, joint, or frame name "
              << "in model with name[" << _model->Name()
              << "]."
              << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
/// \brief Helper function for checking validity of pose relative_to attribute
/// values in the model scope.
/// \param[in] _world World object that contains object _t in its scope.
/// \param[in] _t Object whose pose relative_to attribute is to be checked
/// (may be Model, Frame, Light, etc.).
/// \param[in] _typeName Name of type T for use in console messages.
/// \return True if relative_to attribute values a valid.
template <typename T>
bool checkWorldPoseRelativeTo(
    const sdf::World *_world,
    const T *_t,
    const std::string &_typeName)
{
  const std::string &relativeTo = _t->PoseRelativeTo();

  // the relative_to attribute is always permitted to be empty
  if (relativeTo.empty())
  {
    return true;
  }

  if (relativeTo == _t->Name())
  {
    std::cerr << "Error: relative_to name[" << relativeTo
              << "] is identical to " << _typeName << " name[" << _t->Name()
              << "], causing a graph cycle "
              << "in world with name[" << _world->Name()
              << "]."
              << std::endl;
    return false;
  }
  else if (!_world->ModelNameExists(relativeTo) &&
           !_world->FrameNameExists(relativeTo))
  {
    std::cerr << "Error: relative_to name[" << relativeTo
              << "] specified by " << _typeName << " with name[" << _t->Name()
              << "] does not match a model or frame name "
              << "in world with name[" << _world->Name()
              << "]."
              << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
/// \brief Check that for each pose, the relative_to attribute value
/// does not match its own frame name (for the poses of explicit and
/// implicit frames) but does match the name of a frame in the current
/// scope if the attribute is set and not empty.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all poses have valid relative_to attributes.
bool checkPoseRelativeToNames(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelPoseRelativeToNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t l = 0; l < _model->LinkCount(); ++l)
    {
      auto link = _model->LinkByIndex(l);

      modelResult = checkModelPoseRelativeTo<sdf::Link>(_model, link, "link")
                    && modelResult;
    }
    for (uint64_t j = 0; j < _model->JointCount(); ++j)
    {
      auto joint = _model->JointByIndex(j);

      modelResult = checkModelPoseRelativeTo<sdf::Joint>(_model, joint, "joint")
                    && modelResult;
    }
    for (uint64_t f = 0; f < _model->FrameCount(); ++f)
    {
      auto frame = _model->FrameByIndex(f);

      modelResult = checkModelPoseRelativeTo<sdf::Frame>(_model, frame, "frame")
                    && modelResult;
    }
    return modelResult;
  };

  auto checkWorldPoseRelativeToNames = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    for (uint64_t m = 0; m < _world->ModelCount(); ++m)
    {
      auto model = _world->ModelByIndex(m);

      worldResult = checkWorldPoseRelativeTo<sdf::Model>(_world, model, "model")
                    && worldResult;
    }
    for (uint64_t f = 0; f < _world->FrameCount(); ++f)
    {
      auto frame = _world->FrameByIndex(f);

      worldResult = checkWorldPoseRelativeTo<sdf::Frame>(_world, frame, "frame")
                    && worldResult;
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelPoseRelativeToNames(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    result = checkWorldPoseRelativeToNames(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelPoseRelativeToNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief Check that all joints in contained models have specify parent
/// and child link names that match the names of sibling links.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all models have joints with valid parent and child
/// link names.
bool checkJointParentChildLinkNames(const sdf::Root &_root)
{
  bool result = true;

  auto checkModelJointParentChildNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t j = 0; j < _model->JointCount(); ++j)
    {
      auto joint = _model->JointByIndex(j);

      const std::string &parentName = joint->ParentLinkName();
      if (parentName != "world" && !_model->LinkNameExists(parentName))
      {
        std::cerr << "Error: parent link with name[" << parentName
                  << "] specified by joint with name[" << joint->Name()
                  << "] not found in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      const std::string &childName = joint->ChildLinkName();
      if (childName != "world" && !_model->LinkNameExists(childName))
      {
        std::cerr << "Error: child link with name[" << childName
                  << "] specified by joint with name[" << joint->Name()
                  << "] not found in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      if (childName == parentName)
      {
        std::cerr << "Error: joint with name[" << joint->Name()
                  << "] in model with name[" << _model->Name()
                  << "] must specify different link names for "
                  << "parent and child, while [" << childName
                  << "] was specified for both."
                  << std::endl;
        modelResult = false;
      }
    }
    return modelResult;
  };

  for (uint64_t m = 0; m < _root.ModelCount(); ++m)
  {
    auto model = _root.ModelByIndex(m);
    result = checkModelJointParentChildNames(model) && result;
  }

  for (uint64_t w = 0; w < _root.WorldCount(); ++w)
  {
    auto world = _root.WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelJointParentChildNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief Check that all sibling elements of the any type have unique names.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first duplicate name is found.
/// \param[in] _elem sdf Element to check recursively.
/// \return True if all contained elements have do not share a name with
/// sibling elements of any type.
bool recursiveSiblingUniqueNames(sdf::ElementPtr _elem)
{
  bool result = _elem->HasUniqueChildNames();
  if (!result)
  {
    std::cerr << "Non-unique names detected in "
              << _elem->ToString("")
              << std::endl;
    result = false;
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSiblingUniqueNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
/// \brief Check that all sibling elements of the same type have unique names.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first duplicate name is found.
/// \param[in] _elem sdf Element to check recursively.
/// \return True if all contained elements have do not share a name with
/// sibling elements of the same type.
bool recursiveSameTypeUniqueNames(sdf::ElementPtr _elem)
{
  bool result = true;
  auto typeNames = _elem->GetElementTypeNames();
  for (const std::string &typeName : typeNames)
  {
    if (!_elem->HasUniqueChildNames(typeName))
    {
      std::cerr << "Non-unique names detected in type "
                << typeName << " in\n"
                << _elem->ToString("")
                << std::endl;
      result = false;
    }
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSameTypeUniqueNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE int cmdCheck(const char *_path)
{
  int result = 0;

  sdf::Root root;
  sdf::Errors errors = root.Load(_path);
  if (!errors.empty())
  {
    for (auto &error : errors)
    {
      std::cerr << "Error: " << error.Message() << std::endl;
    }
    result = -1;
  }

  if (!checkCanonicalLinkNames(root))
  {
    std::cerr << "Error: invalid canonical link name.\n";
    result = -1;
  }

  if (!checkJointParentChildLinkNames(root))
  {
    std::cerr << "Error: invalid parent or child link name.\n";
    result = -1;
  }
  if (!checkFrameAttachedToNames(root))
  {
    std::cerr << "Error: invalid frame attached_to name.\n";
    result = -1;
  }
  if (!checkPoseRelativeToNames(root))
  {
    std::cerr << "Error: invalid pose relative_to name.\n";
    result = -1;
  }
  if (!recursiveSiblingUniqueNames(root.Element()))
  {
    std::cerr << "Error: non-unique names detected.\n";
    result = -1;
  }

  if (!checkKinematicGraph(root))
  {
    std::cerr << "Error: invalid kinematic graph.\n";
    result = -1;
  }

  if (!checkFrameAttachedToGraph(root))
  {
    std::cerr << "Error: invalid frame attached_to graph.\n";
    result = -1;
  }

  if (!checkPoseRelativeToGraph(root))
  {
    std::cerr << "Error: invalid pose relative_to graph.\n";
    result = -1;
  }

  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    result = -1;
  }

  sdf::SDFPtr sdf(new sdf::SDF());

  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    result = -1;
  }

  if (!sdf::readFile(_path, sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed.\n";
    result = -1;
  }

  if (result == 0)
  {
    std::cout << "Valid.\n";
  }
  return result;
}

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE char *ignitionVersion()
{
#ifdef _MSC_VER
  return _strdup(SDF_VERSION_FULL);
#else
  return strdup(SDF_VERSION_FULL);
#endif
}

//////////////////////////////////////////////////
/// \brief Print the full description of the SDF spec.
/// \return 0 on success, -1 if SDF could not be initialized.
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE int cmdDescribe()
{
  sdf::SDFPtr sdf(new sdf::SDF());

  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    return -1;
  }

  sdf->PrintDescription();

  return 0;
}

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE int cmdPrint(const char *_path)
{
  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    return -1;
  }

  sdf::SDFPtr sdf(new sdf::SDF());

  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    return -1;
  }

  if (!sdf::readFile(_path, sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed.\n";
    return -1;
  }

  sdf->PrintValues();

  return 0;
}
