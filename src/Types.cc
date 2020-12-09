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

/////////////////////////////////////////////////
std::ostream &operator<<(std::ostream &_out, const sdf::Errors &_errs)
{
  for (const auto &e : _errs)
  {
    _out << e << std::endl;
  }
  return _out;
}

// Split a given absolute name into the parent model name and the local name.
// If the give name is not scoped, this will return an empty string for the
// parent model name and the given name as the local name.
std::pair<std::string, std::string> SplitName(
    const std::string &_absoluteName)
{
  const auto pos = _absoluteName.rfind(kSdfScopeDelimiter);
  if (pos != std::string::npos)
  {
    const std::string first = _absoluteName.substr(0, pos);
    const std::string second =
        _absoluteName.substr(pos + kSdfScopeDelimiter.size());
    return {first, second};
  }
  return {"", _absoluteName};
}

// Join an scope name prefix with a local name using the scope delimeter
std::string JoinName(
    const std::string &_scopeName, const std::string &_localName)
{
  if (_scopeName.empty())
    return _localName;
  return _scopeName + kSdfScopeDelimiter + _localName;
}
}
}
