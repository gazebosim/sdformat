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

#include "sdf/sdf_config.h"
#include "sdf/Filesystem.hh"
#include "sdf/ign.hh"
#include "sdf/parser.hh"
#include "sdf/system_util.hh"

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE int cmdCheck(const char *_path)
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

  std::cout << "Valid.\n";
  return 0;
}

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE char *ignitionVersion()
{
  return sdf_strdup(SDF_VERSION_FULL);
}
