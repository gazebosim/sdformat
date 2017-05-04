/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include <string>
#include <vector>

#include "sdf/Types.hh"

namespace sdf
{
/////////////////////////////////////////////////
std::vector<std::string> split(const std::string &_str,
                               const std::string &_splitter)
{
  std::vector<std::string> ret;
  size_t next = 0;
  size_t current = next;

  if (_splitter.empty())
  {
    // If the splitter is blank, just return the original
    ret.push_back(_str);
    return ret;
  }

  while (next != std::string::npos)
  {
    next = _str.find(_splitter, current);
    ret.push_back(_str.substr(current, next - current));
    current = next + _splitter.length();
  }

  return ret;
}

//////////////////////////////////////////////////
std::string trim(const char *_in)
{
  std::string str(_in);

  const size_t strBegin = str.find_first_not_of(" \t");
  if (strBegin == std::string::npos)
  {
    return "";
  }

  const size_t strRange = str.find_last_not_of(" \t") - strBegin + 1;

  return str.substr(strBegin, strRange);
}
}
