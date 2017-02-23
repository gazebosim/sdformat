/*
 * Copyright 2014 Open Source Robotics Foundation
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
    << "<model name='model'>"
    << "  <plugin name='example' filename='libexample.so'>"
    << "    <value1>true</value1>"
    << "    <value2>1</value2>"
    << "    <value3>false</value3>"
    << "    <value4>0</value4>"
    << "  </plugin>"
    << "</model>"
    << "</sdf>";
  return stream.str();
}

////////////////////////////////////////
// make sure that we can read boolean values from inside a plugin
TEST(PluginBool, ParseBoolValue)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string(), model));

  sdf::ElementPtr plugin =
    model->Root()->GetElement("model")->GetElement("plugin");

  ASSERT_TRUE(plugin->HasElement("value1"));
  EXPECT_TRUE(plugin->Get<bool>("value1"));

  ASSERT_TRUE(plugin->HasElement("value2"));
  EXPECT_TRUE(plugin->Get<bool>("value2"));

  ASSERT_TRUE(plugin->HasElement("value3"));
  EXPECT_FALSE(plugin->Get<bool>("value3"));

  ASSERT_TRUE(plugin->HasElement("value4"));
  EXPECT_FALSE(plugin->Get<bool>("value4"));
}
