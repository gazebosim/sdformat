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

#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

/////////////////////////////////////////////////
TEST(CategoryBitmask, WasSpecified)
{
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <model name="asdf_model">
      <link name="asdf_link">
        <collision name="asdf_collision">
          <geometry/>
          <surface>
            <contact>
              <category_bitmask>1234</category_bitmask>
            </contact>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>)";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  ASSERT_TRUE(sdf::convertString(xmlString, "1.6", sdf));
  ASSERT_NE(nullptr, sdf->Root());
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model")
      ->GetElement("link"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model")
      ->GetElement("link")->GetElement("collision")->GetElement("surface"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model")
      ->GetElement("link")->GetElement("collision")->GetElement("surface")
      ->GetElement("contact"));

  sdf::ElementPtr contact = sdf->Root()->GetElement("world")
    ->GetElement("model") ->GetElement("link")->GetElement("collision")
    ->GetElement("surface")->GetElement("contact");
  ASSERT_NE(nullptr, contact);

  EXPECT_TRUE(contact->HasElement("category_bitmask"));
  EXPECT_EQ(1234, contact->Get<int>("category_bitmask"));
}

/////////////////////////////////////////////////
TEST(CategoryBitmask, WasNotSpecified)
{
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <model name="asdf_model">
      <link name="asdf_link">
        <collision name="asdf_collision">
          <geometry/>
          <surface>
            <contact/>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>)";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  ASSERT_TRUE(sdf::convertString(xmlString, "1.6", sdf));
  ASSERT_NE(nullptr, sdf->Root());
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model")
      ->GetElement("link"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model")
      ->GetElement("link")->GetElement("collision")->GetElement("surface"));
  ASSERT_NE(nullptr, sdf->Root()->GetElement("world")->GetElement("model")
      ->GetElement("link")->GetElement("collision")->GetElement("surface")
      ->GetElement("contact"));

  sdf::ElementPtr contact = sdf->Root()->GetElement("world")
    ->GetElement("model") ->GetElement("link")->GetElement("collision")
    ->GetElement("surface")->GetElement("contact");
  ASSERT_NE(nullptr, contact);

  EXPECT_FALSE(contact->HasElement("category_bitmask"));
}
