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

#include <gtest/gtest.h>
#include <string>
#include "sdf/sdf.hh"

#include "test_config.h"

std::string get_sdf_string()
{
  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name=\"test\">"
    << "  <link name=\"test1\">"
    << "    <visual name=\"bad\">"
    << "      <geometry>"
    << "        <box><size>1 1 1</size></box>"
    << "      </geometry_typo_bad>"
    << "    </visual>"
    << "    <visual name=\"good\">"
    << "      <geometry>"
    << "        <box><size>1 1 1</size></box>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "</model>"
    << "</sdf>";
  return stream.str();
}

////////////////////////////////////////
// make sure that XML errors get caught
TEST(ParserErrorDetection, BadXML)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_FALSE(sdf::readString(get_sdf_string(), model));
}
