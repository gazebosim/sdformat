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
#include "sdf/Joint.hh"
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
/// \brief Check that all joints in contained models have specify parent
/// and child link names that match the names of sibling links.
/// This checks recursively and should check the files exhaustively
/// rather than terminating early when the first error is found.
/// \param[in] _root sdf Root object to check recursively.
/// \return True if all models have joints with valid parent and child
/// link names.
bool checkJointParentChildLinksExist(const sdf::Root &_root)
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
  sdf::Root root;
  sdf::Errors errors = root.Load(_path);
  if (!errors.empty())
  {
    for (auto &error : errors)
    {
      std::cerr << "Error: " << error.Message() << std::endl;
    }
    return -1;
  }

  if (!checkCanonicalLinkNames(root))
  {
    std::cerr << "Error: invalid canonical link name.\n";
    return -1;
  }

  if (!checkJointParentChildLinksExist(root))
  {
    std::cerr << "Error: invalid parent or child link name.\n";
    return -1;
  }

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

  if (!recursiveSiblingUniqueNames(sdf->Root()))
  {
    std::cerr << "Error: non-unique names detected.\n";
    return -1;
  }

  std::cout << "Valid.\n";
  return 0;
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
