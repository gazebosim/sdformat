/*
 * Copyright 2015-2017 Open Source Robotics Foundation
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

#ifdef _WIN32
#include <Windows.h>
#endif

#include <string>
#include <vector>

namespace sdf
{
/////////////////////////////////////////////////
#ifdef _WIN32
const char *winGetEnv(const char *_name)
{
  const DWORD buffSize = 65535;
  static char buffer[buffSize];
  if (GetEnvironmentVariable(_name, buffer, buffSize))
  {
    return buffer;
  }
  return NULL;
}
#else
const char *winGetEnv(const char * /*_name*/)
{
  return NULL;
}
#endif

/////////////////////////////////////////////////
std::vector<std::string> split(const std::string& str,
                               const std::string& splitter)
{
  std::vector<std::string> ret;
  size_t next = 0;
  size_t current = next;

  while (next != std::string::npos)
  {
    next = str.find(splitter, current);
    ret.push_back(str.substr(current, next - current));
    current = next + splitter.length();
  }

  return ret;
}

//////////////////////////////////////////////////
std::string trim(const char *in)
{
  std::string str(in);

  const size_t strBegin = str.find_first_not_of(" \t");
  if (strBegin == std::string::npos)
  {
    return "";  // no content
  }

  const size_t strRange = str.find_last_not_of(" \t") - strBegin + 1;

  return str.substr(strBegin, strRange);
}
}
