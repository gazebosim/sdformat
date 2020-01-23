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
#include "sdf/parser.hh"
#include "sdf/Element.hh"
#include "test_config.h"

/////////////////////////////////////////////////
TEST(Parser, initStringTrim)
{
  sdf::SDFPtr sdf(new sdf::SDF());
  std::ostringstream stream;
  stream << "<element name=\"visual\" required=\"*\" "
         <<          "type=\"string\" default=\"_default_value_\">"
         << "  <attribute name=\"name\" type=\"string\" required=\" 1\""
         << "             default=\"__default__\">"
         << "    <description>test</description>"
         << "  </attribute>"
         << "</element>";

  EXPECT_TRUE(sdf::initString(stream.str(), sdf));
  sdf::ElementPtr root = sdf->Root();
  EXPECT_TRUE(root != nullptr);

  EXPECT_EQ("visual", root->GetName());
  EXPECT_EQ("*", root->GetRequired());
  sdf::ParamPtr value = root->GetValue();
  ASSERT_NE(nullptr, value);
  EXPECT_EQ("string", value->GetTypeName());
  EXPECT_EQ("_default_value_", value->GetDefaultAsString());

  sdf::ParamPtr attr = root->GetAttribute("name");
  EXPECT_TRUE(attr != nullptr);
  EXPECT_TRUE(attr->GetRequired());
}

/////////////////////////////////////////////////
/// Tests whether the input sdf string satisfies the unique name criteria among
/// same types
sdf::SDFPtr InitSDF()
{
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  return sdf;
}

/////////////////////////////////////////////////
TEST(Parser, ReusedSDFVersion)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";
  const std::string path17 = pathBase +"/model_link_relative_to.sdf";
  const std::string path16 = pathBase +"/joint_complete.sdf";

  // Call readFile API that always converts
  sdf::SDFPtr sdf = InitSDF();
  EXPECT_TRUE(sdf::readFile(path17, sdf));
  EXPECT_EQ("1.7", sdf->Root()->Get<std::string>("version"));
  EXPECT_EQ("1.7", sdf->OriginalVersion());
  EXPECT_EQ("1.7", sdf->Root()->OriginalVersion());

  sdf->Clear();

  EXPECT_TRUE(sdf::readFile(path16, sdf));
  EXPECT_EQ("1.7", sdf->Root()->Get<std::string>("version"));
  EXPECT_EQ("1.6", sdf->OriginalVersion());
  EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
}

/////////////////////////////////////////////////
TEST(Parser, NameUniqueness)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  // These tests are copies of the ones in ign_TEST.cc but use direct calls to
  // name uniqueness validator functions instead of going through ign.

  // Check an SDF file with sibling elements of the same type (world)
  // that have duplicate names.
  {
    std::string path = pathBase +"/world_duplicate.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSameTypeUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with sibling elements of different types (model, light)
  // that have duplicate names.
  {
    std::string path = pathBase +"/world_sibling_same_names.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSiblingUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with sibling elements of the same type (link)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_duplicate_links.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSameTypeUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with sibling elements of the same type (joint)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_duplicate_joints.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSameTypeUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with sibling elements of different types (link, joint)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_link_joint_same_name.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSiblingUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with sibling elements of the same type (collision)
  // that have duplicate names.
  {
    std::string path = pathBase +"/link_duplicate_sibling_collisions.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSameTypeUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with sibling elements of the same type (visual)
  // that have duplicate names.
  {
    std::string path = pathBase +"/link_duplicate_sibling_visuals.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_FALSE(sdf::recursiveSiblingUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with cousin elements of the same type (collision)
  // that have duplicate names. This is a valid file.
  {
    std::string path = pathBase +"/link_duplicate_cousin_collisions.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_TRUE(sdf::recursiveSameTypeUniqueNames(sdf->Root()));
    EXPECT_TRUE(sdf::recursiveSiblingUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Check an SDF file with cousin elements of the same type (visual)
  // that have duplicate names. This is a valid file.
  {
    std::string path = pathBase +"/link_duplicate_cousin_visuals.sdf";
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_TRUE(sdf::recursiveSameTypeUniqueNames(sdf->Root()));
    EXPECT_TRUE(sdf::recursiveSiblingUniqueNames(sdf->Root()));
    EXPECT_EQ(path, sdf->FilePath());
    EXPECT_EQ(path, sdf->Root()->FilePath());
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
