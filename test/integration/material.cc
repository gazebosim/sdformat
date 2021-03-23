/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include <string>
#include <gtest/gtest.h>

#include <ignition/math/Color.hh>

#include "sdf/sdf.hh"
#include "test_config.h"

void ExpectInvalidWithMessage(sdf::Errors &_errors, std::string _compType)
{
  for (const auto &e : _errors)
  {
    if (e.Message().find(_compType) != std::string::npos)
    {
      EXPECT_EQ(e.Code(), sdf::ErrorCode::ELEMENT_INVALID);
      break;
    }
  }
}

//////////////////////////////////////////////////
TEST(Material, InvalidColors)
{
  std::string testFile =
      sdf::testing::TestFile("sdf", "material_invalid.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  std::cout << errors << std::endl;

  ASSERT_FALSE(errors.empty());
  ExpectInvalidWithMessage(errors, "ambient");

  // since the above test will break at //ambient, it doesn't test the other
  // invalid cases. Below tests these other invalid cases

  // less than 3 values
  std::string testSDF = R"(
    <?xml version="1.0" ?>
    <sdf version="1.8">
      <model name="model">
        <link name="link">
          <visual name="material">
            <material>
              <specular>0 0.1</specular>
            </material>
          </visual>
        </link>
      </model>
    </sdf>)";

  errors.clear();
  errors = root.LoadSdfString(testSDF);
  std::cout << errors << std::endl;

  ASSERT_FALSE(errors.empty());
  ExpectInvalidWithMessage(errors, "specular");

  // negative value
  testSDF = R"(
    <?xml version="1.0" ?>
    <sdf version="1.8">
    <model name="model">
      <link name="link">
        <visual name="material">
          <material>
            <emissive>0.1 0.2    -1</emissive>
          </material>
        </visual>
      </link>
    </model>
    </sdf>)";

  errors.clear();
  errors = root.LoadSdfString(testSDF);
  std::cout << errors << std::endl;

  ASSERT_FALSE(errors.empty());
  ExpectInvalidWithMessage(errors, "emissive");

  // more than 4 values
  testSDF = R"(
    <?xml version="1.0" ?>
    <sdf version="1.8">
    <model name="model">
      <link name="link">
        <visual name="material">
          <material>
            <diffuse>0.1 0.2 0.3 0.4 0.5</diffuse>
          </material>
        </visual>
      </link>
    </model>
    </sdf>)";

  errors.clear();
  errors = root.LoadSdfString(testSDF);
  std::cout << errors << std::endl;

  ASSERT_FALSE(errors.empty());
  ExpectInvalidWithMessage(errors, "diffuse");

  // invalid string value
  testSDF = R"(
    <?xml version="1.0" ?>
    <sdf version="1.8">
    <model name="model">
      <link name="link">
        <visual name="material">
          <material>
            <ambient>0.1 0.2 test</ambient>
          </material>
        </visual>
      </link>
    </model>
    </sdf>)";

  errors.clear();
  errors = root.LoadSdfString(testSDF);
  std::cout << errors << std::endl;

  ASSERT_FALSE(errors.empty());
  ExpectInvalidWithMessage(errors, "ambient");
}

//////////////////////////////////////////////////
TEST(Material, ValidColors)
{
  std::string testFile =
      sdf::testing::TestFile("sdf", "material_valid.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  std::cout << errors << std::endl;

  ASSERT_TRUE(errors.empty());

  sdf::ElementPtr elem = root.Element()->GetElement("model")
                                       ->GetElement("link")
                                       ->GetElement("visual")
                                       ->GetElement("material");
  ASSERT_NE(elem, nullptr);

  EXPECT_EQ(elem->Get<ignition::math::Color>("diffuse"),
            ignition::math::Color(0, 0.1f, 0.2f, 1));
  EXPECT_EQ(elem->Get<ignition::math::Color>("specular"),
            ignition::math::Color(0, 0.1f, 0.2f, 0.3f));
  EXPECT_EQ(elem->Get<ignition::math::Color>("emissive"),
            ignition::math::Color(0.12f, 0.23f, 0.34f, 0.56f));
  EXPECT_EQ(elem->Get<ignition::math::Color>("ambient"),
            ignition::math::Color(0, 0, 0, 1));
}
