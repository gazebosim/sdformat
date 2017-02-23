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
    << "  <link name='link'/>"
    << "  <plugin name='example' filename='libexample.so'>"
    << "    <user attribute='attribute' />"
    << "    <value attribute='attribute'>value</value>"
    << "  </plugin>"
    << "</model>"
    << "</sdf>";
  return stream.str();
}

////////////////////////////////////////
// make sure that plugin attributes get parsed
TEST(PluginAttribute, ParseAttributes)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string(), model));

  sdf::ElementPtr plugin =
    model->Root()->GetElement("model")->GetElement("plugin");
  ASSERT_TRUE(plugin->HasElement("user"));
  {
    sdf::ElementPtr user = plugin->GetElement("user");
    EXPECT_TRUE(user->HasAttribute("attribute"));
    EXPECT_EQ(user->GetAttribute("attribute")->GetAsString(),
              std::string("attribute"));
  }
  {
    sdf::ElementPtr value = plugin->GetElement("value");
    EXPECT_TRUE(value->HasAttribute("attribute"));
    EXPECT_EQ(value->GetAttribute("attribute")->GetAsString(),
              std::string("attribute"));
    EXPECT_EQ(value->Get<std::string>(""),
              std::string("value"));
  }
}
