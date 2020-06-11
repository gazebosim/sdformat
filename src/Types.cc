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

#include <locale>
#include <string>
#include <vector>

#include "sdf/Types.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

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
  return sdf::trim(std::string(_in));
}

<<<<<<< HEAD
//////////////////////////////////////////////////
std::string trim(const std::string &_in)
{
  const size_t strBegin = _in.find_first_not_of(" \t\n");
  if (strBegin == std::string::npos)
  {
    return "";
  }

  const size_t strRange = _in.find_last_not_of(" \t\n") - strBegin + 1;

  return _in.substr(strBegin, strRange);
}

/////////////////////////////////////////////////
std::string lowercase(const std::string &_in)
{
  std::string out = _in;
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = std::tolower(out[i], std::locale());
  return out;
}
}
}
