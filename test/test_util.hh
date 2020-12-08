/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef SDF_TEST_UTIL_HH_
#define SDF_TEST_UTIL_HH_

#include "sdf/Filesystem.hh"

#include "test_config.h"

#define SDF_TMP_DIR "tmp-sdf/"

namespace sdf
{
  namespace testing
  {
    bool TestDataPath(std::string &_dataDir)
    {
      if(char* dataDir = std::getenv("TEST_SRCDIR"))
      {
        _dataDir = sdf::filesystem::append(dataDir, "__main__/sdformat");
        return true;
      }
      else
      {
        _dataDir = PROJECT_SOURCE_PATH;
        return true;
      }

      return false;
    }

    bool TestTmpPath(std::string &_tmpDir)
    {
      if (const char* tmpDir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR"))
      {
        _tmpDir = tmpDir;
        return true;
      }

      if (char* homeDir = std::getenv("HOME"))
      {
        _tmpDir = sdf::filesystem::append(homeDir, SDF_TMP_DIR);
        return true;
      }

      return false;
    }
  }  // namespace testing
}  // namespace sdf

#endif  // SDF_TEST_UTIL_HH_

