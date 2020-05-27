/*
 * Copyright 2020 Open Source Robotics Foundation
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
#include "XmlUtils.hh"

#include <algorithm>

#include "sdf/Console.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
//*
tinyxml2::XMLNode* DeepClone(tinyxml2::XMLDocument *_doc, const tinyxml2::XMLNode *_src)
{
  if (_src == nullptr)
  {
    sdferr << "Pointer to XML node _src is NULL\n";
    return NULL;
  }

  tinyxml2::XMLNode *copy = _src->ShallowClone(_doc);
  if (copy == nullptr)
  {
    sdferr << "Failed to clone node " << _src->Value() << "\n";
    return nullptr;
  }

  for (const tinyxml2::XMLNode *node = _src->FirstChild(); node != nullptr;
       node = node->NextSibling())
  {
    auto* child_copy = DeepClone(_doc, node);
    if(child_copy == nullptr)
    {
      sdferr << "Failed to clone child " << node->Value() << "\n";
      return nullptr;
    }
    copy->InsertEndChild(child_copy);
  }

  return copy;
}

/////////////////////////////////////////////////
std::string &TrimStringLeft(std::string &_s)
{
  _s.erase(_s.begin(),find_if_not(_s.begin(),_s.end(),[](int c){return isspace(c);}));
  return _s;
}

/////////////////////////////////////////////////
std::string &TrimStringRight(std::string &_s)
{
  _s.erase(find_if_not(_s.rbegin(),_s.rend(),[](int c){return isspace(c);}).base(), _s.end());
  return _s;
}

/////////////////////////////////////////////////
std::string TrimString(const std::string &_s)
{
  std::string t = _s;
  return TrimStringLeft(TrimStringRight(t));
}
}
}

