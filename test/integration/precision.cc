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
#include "sdf/parser.hh"

std::string get_sdf_string()
{
  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name=\"test\">"
    << "  <link name=\"link\">"
    << "    <visual name=\"visual\">"
    << "      <geometry>"
    << "        <sphere><radius>182536.87</radius></sphere>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "</model>"
    << "</sdf>";
  return stream.str();
}

////////////////////////////////////////
// make sure that XML errors get caught
TEST(Precision, StringToDouble)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string(), model));

  sdf::ElementPtr modelElem = model->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasElement("visual"));
  sdf::ElementPtr visualElem = linkElem->GetElement("visual");
  EXPECT_TRUE(visualElem->HasElement("geometry"));
  sdf::ElementPtr geomElem = visualElem->GetElement("geometry");
  EXPECT_TRUE(geomElem->HasElement("sphere"));
  sdf::ElementPtr sphereElem = geomElem->GetElement("sphere");
  EXPECT_TRUE(sphereElem->HasElement("radius"));
  sdf::ElementPtr radiusElem = sphereElem->GetElement("radius");

  std::cout << std::setprecision(10) << radiusElem->Get<double>() << std::endl;

  EXPECT_DOUBLE_EQ(radiusElem->Get<double>(), 182536.87);
}
