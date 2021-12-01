/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <ignition/math/Color.hh>
#include "sdf/sdf.hh"

#include "test_config.h"

////////////////////////////////////////
// Test parsing a link element that has a child light element
TEST(Frame, LinkLight)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='link'>"
    << "    <light type= 'point' name='my_light'>"
    << "      <pose>0.1 0 0 0 0 0</pose>"
    << "      <diffuse>0.2 0.3 0.4 1</diffuse>"
    << "      <specular>0.3 0.4 0.5 1</specular>"
    << "    </light>"
    << "  </link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "my_model");

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // light
  EXPECT_TRUE(linkElem->HasElement("light"));
  sdf::ElementPtr lightElem = linkElem->GetElement("light");
  EXPECT_TRUE(lightElem->HasAttribute("name"));
  EXPECT_EQ(lightElem->Get<std::string>("name"), "my_light");
  EXPECT_TRUE(lightElem->HasAttribute("type"));
  EXPECT_EQ(lightElem->Get<std::string>("type"), "point");

  EXPECT_TRUE(lightElem->HasElement("pose"));
  auto pose = lightElem->Get<ignition::math::Pose3d>("pose");
  EXPECT_EQ(ignition::math::Pose3d(0.1, 0, 0, 0, 0, 0), pose);

  // diffuse
  EXPECT_TRUE(lightElem->HasElement("diffuse"));
  sdf::ElementPtr diffuseElem = lightElem->GetElement("diffuse");
  EXPECT_EQ(diffuseElem->Get<ignition::math::Color>(),
      ignition::math::Color(0.2f, 0.3f, 0.4f, 1.0f));

  // specular
  EXPECT_TRUE(lightElem->HasElement("specular"));
  sdf::ElementPtr specularElem = lightElem->GetElement("specular");
  EXPECT_EQ(specularElem->Get<ignition::math::Color>(),
      ignition::math::Color(0.3f, 0.4f, 0.5f, 1.0f));
}
