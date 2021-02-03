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

#include <sstream>
#include <fstream>
#include <cstdlib>
#include <gtest/gtest.h>
#include "sdf/parser.hh"
#include "sdf/Element.hh"
#include "sdf/Console.hh"
#include "sdf/Filesystem.hh"
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
/// Checks emitted warnings for custom/unknown elements in log file
TEST(Parser, CustomUnknownElements)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";
  const std::string path = pathBase +"/custom_and_unknown_elements.sdf";

  sdf::SDFPtr sdf = InitSDF();
  EXPECT_TRUE(sdf::readFile(path, sdf));

#ifndef _WIN32
  char *homeDir = getenv("HOME");
#else
  char *homeDir;
  size_t sz = 0;
  _dupenv_s(&homeDir, &sz, "HOMEPATH");
#endif

  std::string pathLog =
    sdf::filesystem::append(homeDir, ".sdformat", "sdformat.log");

  std::fstream fs;
  fs.open(pathLog);
  ASSERT_TRUE(fs.is_open());

  std::stringstream fileStr;
  fs >> fileStr.rdbuf();

  EXPECT_NE(fileStr.str().find("XML Element[test_unknown]"), std::string::npos);
  EXPECT_EQ(fileStr.str().find("XML Element[test:custom]"), std::string::npos);
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
  EXPECT_EQ(SDF_PROTOCOL_VERSION, sdf->Root()->Get<std::string>("version"));
  EXPECT_EQ("1.7", sdf->OriginalVersion());
  EXPECT_EQ("1.7", sdf->Root()->OriginalVersion());

  sdf->Clear();

  EXPECT_TRUE(sdf::readFile(path16, sdf));
  EXPECT_EQ(SDF_PROTOCOL_VERSION, sdf->Root()->Get<std::string>("version"));
  EXPECT_EQ("1.6", sdf->OriginalVersion());
  EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
}

/////////////////////////////////////////////////
TEST(Parser, readFileConversions)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";
  const std::string path = pathBase +"/joint_complete.sdf";

  // Call readFile API that always converts
  {
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFile(path, sdf));
    EXPECT_EQ(SDF_PROTOCOL_VERSION, sdf->Root()->Get<std::string>("version"));
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }

  // Call readFile API that does not convert
  {
    sdf::Errors errors;
    sdf::SDFPtr sdf = InitSDF();
    EXPECT_TRUE(sdf::readFileWithoutConversion(path, sdf, errors));
    EXPECT_EQ("1.6", sdf->Root()->Get<std::string>("version"));
    EXPECT_EQ("1.6", sdf->OriginalVersion());
    EXPECT_EQ("1.6", sdf->Root()->OriginalVersion());
  }
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
/// Check that _a contains _b
static bool contains(const std::string &_a, const std::string &_b)
{
  return _a.find(_b) != std::string::npos;
}

/////////////////////////////////////////////////
TEST(Parser, SyntaxErrorInValues)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(false);
#endif

  {
    std::string path = pathBase +"/bad_syntax_pose.sdf";
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(contains, buffer.str(),
                 "Unable to set value [bad 0 0 0 0 0 ] for key[pose]");
  }
  {
    // clear the contents of the buffer
    buffer.str("");
    std::string path = pathBase +"/bad_syntax_double.sdf";
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(contains, buffer.str(),
                 "Unable to set value [bad ] for key[linear]");
  }
  {
    // clear the contents of the buffer
    buffer.str("");
    std::string path = pathBase +"/bad_syntax_vector.sdf";
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(contains, buffer.str(),
                 "Unable to set value [0 1 bad ] for key[gravity]");
  }

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(true);
#endif
}

TEST(Parser, PlacementFrameMissingPose)
{
  const std::string modelRootPath = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "integration", "model");

  const std::string testModelPath = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "sdf", "placement_frame_missing_pose.sdf");

  sdf::setFindCallback([&](const std::string &_file)
      {
        return sdf::filesystem::append(modelRootPath, _file);
      });
  sdf::SDFPtr sdf = InitSDF();
  sdf::Errors errors;
  EXPECT_FALSE(sdf::readFile(testModelPath, sdf, errors));
  ASSERT_GE(errors.size(), 0u);
  EXPECT_EQ(sdf::ErrorCode::MODEL_PLACEMENT_FRAME_INVALID, errors[0].Code());
}

/////////////////////////////////////////////////
/// Fixture for setting up stream redirection
class ValueConstraintsFixture : public ::testing::Test
{
  public: ValueConstraintsFixture() = default;

  public: void ClearErrorBuffer()
  {
    this->errBuffer.str("");
  }

  // cppcheck-suppress unusedFunction
  protected: void SetUp() override
  {
    sdf::Console::Instance()->SetQuiet(false);
    oldRdbuf = std::cerr.rdbuf(errBuffer.rdbuf());
  }

  // cppcheck-suppress unusedFunction
  protected: void TearDown() override
  {
    std::cerr.rdbuf(oldRdbuf);
#ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(true);
#endif
  }

  public: std::stringstream errBuffer;
  private: std::streambuf *oldRdbuf;
};

/////////////////////////////////////////////////
/// Check if minimum/maximum values are valided
TEST_F(ValueConstraintsFixture, ElementMinMaxValues)
{
  std::string sdfDescPath = std::string(PROJECT_SOURCE_PATH) +
                            "/test/sdf/stricter_semantics_desc.sdf";

  auto sdfTest = std::make_shared<sdf::SDF>();
  sdf::initFile(sdfDescPath, sdfTest);

  // Initialize the root.sdf description and add our test description as one of
  // its children
  auto sdf = InitSDF();
  sdf->Root()->AddElementDescription(sdfTest->Root());

  auto wrapInSdf = [](std::string _xml) -> std::string
  {
    std::stringstream ss;
    ss << "<sdf version=\"" << SDF_PROTOCOL_VERSION << "\"><test>" << _xml
       << "</test></sdf>";
    return ss.str();
  };

  {
    auto elem = sdf->Root()->Clone();
    sdf::Errors errors;
    EXPECT_TRUE(sdf::readString(
        wrapInSdf("<int_t>0</int_t><double_t>0</double_t>"), elem, errors));
    EXPECT_TRUE(errors.empty()) << errors[0];
  }

  auto errorContains =
      [](const std::string &_expStr, const std::string &_errs)
  {
    return _errs.find(_expStr) != std::string::npos;
  };

  {
    this->ClearErrorBuffer();
    auto elem = sdf->Root()->Clone();
    EXPECT_FALSE(sdf::readString(
        wrapInSdf("<int_t>-1</int_t>"), elem));
    EXPECT_PRED2(errorContains,
                 "The value [-1] is less than the minimum allowed value of [0] "
                 "for key [int_t]",
                 this->errBuffer.str());
  }

  {
    this->ClearErrorBuffer();
    auto elem = sdf->Root()->Clone();
    EXPECT_FALSE(sdf::readString(
        wrapInSdf("<double_t>-1.0</double_t>"), elem));

    EXPECT_PRED2(
        errorContains,
        "The value [-1] is less than the minimum allowed value of [0] for key "
        "[double_t]",
        this->errBuffer.str());
  }

  {
    this->ClearErrorBuffer();
    auto elem = sdf->Root()->Clone();
    EXPECT_FALSE(sdf::readString(wrapInSdf("<int_t>20</int_t>"), elem));
    EXPECT_PRED2(
        errorContains,
        "The value [20] is greater than the maximum allowed value of [10]",
        this->errBuffer.str());
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  // temporarily set HOME to build directory
#ifndef _WIN32
  setenv("HOME", PROJECT_BINARY_DIR, 1);
#else
  std::string buildDir = PROJECT_BINARY_DIR;
  for (int i = 0; i < buildDir.size(); ++i)
  {
    if (buildDir[i] == '/')
      buildDir[i] = '\\';
  }
  std::string homePath = "HOMEPATH=" + buildDir;
  _putenv(homePath.c_str());
#endif
  sdf::Console::Clear();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
