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

#include <gz/utils/Environment.hh>

#include "test_config.hh"
#include "test_utils.hh"

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
  const auto path = sdf::testing::TestFile(
      "sdf", "custom_and_unknown_elements.sdf");

  sdf::SDFPtr sdf = InitSDF();
  EXPECT_TRUE(sdf::readFile(path, sdf));

#ifndef _WIN32
  std::string homeVarName = "HOME";
#else
  std::string homeVarName = "HOMEPATH";
#endif

  std::string homeDir;
  ASSERT_TRUE(gz::utils::env(homeVarName, homeDir));

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
  const auto path17 = sdf::testing::TestFile(
      "sdf", "model_link_relative_to.sdf");
  const auto path16 = sdf::testing::TestFile("sdf", "joint_complete.sdf");

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
  const auto path = sdf::testing::TestFile("sdf", "joint_complete.sdf");

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
  // These tests are copies of the ones in gz_TEST.cc but use direct calls to
  // name uniqueness validator functions instead of going through gz.

  // Check an SDF file with sibling elements of the same type (world)
  // that have duplicate names.
  {
    const auto path = sdf::testing::TestFile("sdf", "world_duplicate.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "world_sibling_same_names.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "model_duplicate_links.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "model_duplicate_joints.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "model_link_joint_same_name.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "link_duplicate_sibling_collisions.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "link_duplicate_sibling_visuals.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "link_duplicate_cousin_collisions.sdf");
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
    const auto path = sdf::testing::TestFile("sdf",
        "link_duplicate_cousin_visuals.sdf");
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
TEST(Parser, SyntaxErrorInValues)
{
  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(false);
#endif

  {
    const auto path = sdf::testing::TestFile("sdf", "bad_syntax_pose.sdf");
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Unable to set value [bad 0 0 0 0 0] for key [pose]");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "bad_syntax_pose.sdf:L5");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/model[@name=\"robot1\"]/pose:");
  }
  {
    // clear the contents of the buffer
    buffer.str("");
    const auto path = sdf::testing::TestFile("sdf", "bad_syntax_double.sdf");
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Unable to set value [bad] for key[linear]");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "bad_syntax_double.sdf:L7");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/model[@name=\"robot1\"]/"
                 "link[@name=\"link\"]/velocity_decay/linear:");
  }
  {
    // clear the contents of the buffer
    buffer.str("");
    const auto path = sdf::testing::TestFile("sdf", "bad_syntax_vector.sdf");
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Unable to set value [0 1 bad ] for key[gravity]");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "bad_syntax_vector.sdf:L4");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/gravity:");
  }

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(true);
#endif
}

TEST(Parser, MissingRequiredAttributesErrors)
{
  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(false);
#endif

  {
    // clear the contents of the buffer
    buffer.str("");

    const auto path = sdf::testing::TestFile("sdf", "box_bad_test.world");
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Error Code " +
                 std::to_string(
                    static_cast<int>(sdf::ErrorCode::ATTRIBUTE_MISSING)));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Required attribute[name] in element[link] is not specified "
                 "in SDF.");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(), "box_bad_test.world:L6");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/model[@name=\"box\"]/link:");
  }

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(true);
#endif
}

TEST(Parser, IncludesErrors)
{
  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(false);
#endif

  {
    // clear the contents of the buffer
    buffer.str("");

    const auto path = sdf::testing::TestFile("sdf", "includes_missing_uri.sdf");
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Error Code " +
                 std::to_string(
                    static_cast<int>(sdf::ErrorCode::ATTRIBUTE_MISSING)));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "<include> element missing 'uri' attribute");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "includes_missing_uri.sdf:L5");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/include[0]:");
  }
  {
    // clear the contents of the buffer
    buffer.str("");

    const auto path =
        sdf::testing::TestFile("sdf", "includes_missing_model.sdf");
    sdf::setFindCallback([&](const std::string &/*_file*/)
        {
          return "";
        });

    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);

    std::cout << buffer.str() << std::endl;

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Error Code " +
                 std::to_string(
                    static_cast<int>(sdf::ErrorCode::URI_LOOKUP)));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Unable to find uri[missing_model]");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "includes_missing_model.sdf:L6");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/include[0]/uri:");
  }
  {
    // clear the contents of the buffer
    buffer.str("");

    const auto path =
        sdf::testing::TestFile("sdf", "includes_model_without_sdf.sdf");
    sdf::setFindCallback([&](const std::string &_file)
        {
          return sdf::testing::TestFile("integration", "model", _file);
        });

    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Error Code " +
                 std::to_string(static_cast<int>(sdf::ErrorCode::URI_LOOKUP)));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Unable to resolve uri[box_missing_config]");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "since it does not contain a model.config");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "includes_model_without_sdf.sdf:L6");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/include[0]/uri:");
  }
  {
    // clear the contents of the buffer
    buffer.str("");

    const auto path =
        sdf::testing::TestFile("sdf", "includes_without_top_level.sdf");
    sdf::setFindCallback([&](const std::string &_file)
        {
          return sdf::testing::TestFile("integration", "model", _file);
        });

    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);

    sdf::readFile(path, sdf);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Error Code " +
                 std::to_string(
                    static_cast<int>(sdf::ErrorCode::ELEMENT_MISSING)));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "Failed to find top level <model> / <actor> / <light> for "
                 "<include>\n");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "includes_without_top_level.sdf:L6");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
                 "/sdf/world[@name=\"default\"]/include[0]/uri:");
  }

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(true);
#endif
}

TEST(Parser, PlacementFrameMissingPose)
{
  const std::string testModelPath =
      sdf::testing::TestFile("sdf", "placement_frame_missing_pose.sdf");

  sdf::setFindCallback([&](const std::string &_file)
      {
        return sdf::testing::TestFile("integration", "model", _file);
      });
  sdf::SDFPtr sdf = InitSDF();
  sdf::Errors errors;
  EXPECT_FALSE(sdf::readFile(testModelPath, sdf, errors));
  ASSERT_GE(errors.size(), 0u);
  EXPECT_EQ(sdf::ErrorCode::MODEL_PLACEMENT_FRAME_INVALID, errors[0].Code());
}

/////////////////////////////////////////////////
// Delimiter '::' in name should error in SDFormat 1.8 but not in 1.7
TEST(Parser, DoubleColonNameAttrError)
{
  sdf::SDFPtr sdf = InitSDF();
  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.8'>"
         << "  <model name='test'>"
         << "    <link name='A::B'/>"
         << "  </model>"
         << "</sdf>";

  sdf::Errors errors;
  EXPECT_FALSE(sdf::readString(stream.str(), sdf, errors));
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::RESERVED_NAME);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Detected delimiter '::' in element name in<link name='A::B'/>"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::RESERVED_NAME);
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "Delimiter '::' found in attribute names of element <sdf>,"
      " which is not allowed in SDFormat >= 1.8"));

  sdf = InitSDF();
  stream.str("");
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.7'>"
         << "  <model name='test::A'/>"
         << "</sdf>";

  errors.clear();
  EXPECT_TRUE(sdf::readString(stream.str(), sdf, errors));
  EXPECT_EQ(errors.size(), 0u);
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

  protected: void SetUp() override
  {
    sdf::Console::Instance()->SetQuiet(false);
    oldRdbuf = std::cerr.rdbuf(errBuffer.rdbuf());
  }

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
  const auto sdfDescPath =
    sdf::testing::TestFile("sdf", "stricter_semantics_desc.sdf");

  auto sdfTest = std::make_shared<sdf::SDF>();
  sdf::initFile(sdfDescPath, sdfTest);

  // Initialize the root.sdf description and add our test description as one of
  // its children
  auto sdf = InitSDF();
  sdf->Root()->AddElementDescription(sdfTest->Root());

  auto wrapInSdf = [](const std::string &_xml) -> std::string
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
/// Check for parsing errors while reading strings
TEST(Parser, ReadSingleLineStringError)
{
  sdf::SDFPtr sdf = InitSDF();
  std::ostringstream stream;
  stream
    << "<sdf version='1.8'>"
    << "<model name=\"test\">"
    << "  <link name=\"test1\">"
    << "    <visual name=\"good\">"
    << "      <geometry>"
    << "        <box><size>1 1 1</size></box>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "  <link>"
    << "    <visual name=\"good2\">"
    << "      <geometry>"
    << "        <box><size>1 1 1</size></box>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "</model>"
    << "</sdf>";
  sdf::Errors errors;
  EXPECT_FALSE(sdf::readString(stream.str(), sdf, errors));
  ASSERT_NE(errors.size(), 0u);

  std::cerr << errors[0] << std::endl;

  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  ASSERT_TRUE(errors[0].FilePath().has_value());
  EXPECT_EQ(errors[0].FilePath().value(), sdf::kSdfStringSource);
  ASSERT_TRUE(errors[0].LineNumber().has_value());
  EXPECT_EQ(errors[0].LineNumber().value(), 1);
}

/////////////////////////////////////////////////
/// Check for parsing errors while reading strings
TEST(Parser, ReadMultiLineStringError)
{
  sdf::SDFPtr sdf = InitSDF();
  std::ostringstream stream;
  stream
    << "<sdf version='1.8'>\n"
    << "<model name=\"test\">\n"
    << "  <link name=\"test1\">\n"
    << "    <visual name=\"good\">\n"
    << "      <geometry>\n"
    << "        <box><size>1 1 1</size></box>\n"
    << "      </geometry>\n"
    << "    </visual>\n"
    << "  </link>\n"
    << "  <link>\n"
    << "    <visual name=\"good2\">\n"
    << "      <geometry>\n"
    << "        <box><size>1 1 1</size></box>\n"
    << "      </geometry>\n"
    << "    </visual>\n"
    << "  </link>\n"
    << "</model>\n"
    << "</sdf>\n";
  sdf::Errors errors;
  EXPECT_FALSE(sdf::readString(stream.str(), sdf, errors));
  ASSERT_NE(errors.size(), 0u);

  std::cerr << errors[0] << std::endl;

  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  ASSERT_TRUE(errors[0].FilePath().has_value());
  EXPECT_EQ(errors[0].FilePath().value(), sdf::kSdfStringSource);
  ASSERT_TRUE(errors[0].LineNumber().has_value());
  EXPECT_EQ(errors[0].LineNumber().value(), 10);
}

/////////////////////////////////////////////////
/// Check that FilePath() for elements loaded from a file returns the file path
/// whereas it returns "" for elements loaded from a string.
TEST(Parser, ElementFilePath)
{
  auto findFileCb = [](const std::string &_uri)
  {
    return sdf::testing::TestFile("integration", "model", _uri);
  };

  const std::string testString = R"(
    <?xml version="1.0" ?>
    <sdf version="1.8">
      <world name="default">
        <include>
          <uri>test_model</uri>
          <name>included_model</name>
        </include>
        <model name="direct_model">
          <link name="link"/>
        </model>
      </world>
    </sdf>)";

  sdf::ParserConfig config;
  config.SetFindCallback(findFileCb);
  sdf::Errors errors;
  sdf::SDFPtr sdf = InitSDF();
  ASSERT_TRUE(sdf::readString(testString, config, sdf, errors));
  ASSERT_TRUE(errors.empty());
  ASSERT_NE(nullptr, sdf);
  auto root = sdf->Root();
  ASSERT_NE(nullptr, root);
  EXPECT_EQ("", root->FilePath());

  // included_model is loaded from a file, so FilePath should return a file
  // path.
  auto includedModel = root->GetElement("world")->GetElement("model");
  EXPECT_EQ("included_model", includedModel->Get<std::string>("name"));
  EXPECT_EQ(findFileCb(sdf::filesystem::append("test_model", "model.sdf")),
      includedModel->FilePath());

  // direct_model is loaded from a string along with the containing world, so
  // FilePath should return "".
  auto directModel = includedModel->GetNextElement("model");
  EXPECT_EQ("direct_model", directModel->Get<std::string>("name"));
  EXPECT_EQ("", directModel->FilePath());
}

/////////////////////////////////////////////////
/// Check for missing element segfault, #590
TEST(Parser, MissingRequiredElement)
{
  const std::string testString = R"(<?xml version="1.0" ?>
    <sdf version="1.8">
      <world name="default">
        <model name="test_model">
          <link name="link_0"/>
          <link name="link_1"/>
          <joint name="joint" type="revolute">
            <parent>link_0</parent>
            <axis>
              <xyz>1 0 0</xyz>
              <limit>
                <lower>-1</lower>
                <upper>1</upper>
              </limit>
            </axis>
          </joint>
        </model>
      </world>
    </sdf>)";

  sdf::Errors errors;
  sdf::SDFPtr sdf = InitSDF();
  EXPECT_FALSE(sdf::readString(testString, sdf, errors));

  ASSERT_NE(errors.size(), 0u);
  std::cerr << errors[0] << std::endl;

  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  ASSERT_TRUE(errors[0].FilePath().has_value());
  EXPECT_EQ(errors[0].FilePath().value(),
      std::string(sdf::kSdfStringSource));
  ASSERT_TRUE(errors[0].LineNumber().has_value());
  EXPECT_EQ(errors[0].LineNumber().value(), 7);
}

TEST(Parser, ElementRemovedAfterDeprecation)
{
  const std::string testString = R"(<?xml version="1.0" ?>
    <sdf version="1.9">
      <model name="test_model">
        <link name="link_0"/>
        <link name="link_1"/>
        <joint name="joint" type="revolute">
          <parent>link_0</parent>
          <child>link_1</child>
          <axis>
            <initial_position>0.1</initial_position> <!-- Removed in 1.9 -->
          </axis>
        </joint>
      </model>
    </sdf>)";

  sdf::Errors errors;
  sdf::SDFPtr sdf = InitSDF();
  sdf::ParserConfig config;
  config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::ERR);
  sdf::readString(testString, config, sdf, errors);

  ASSERT_NE(errors.size(), 0u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  ASSERT_TRUE(errors[0].LineNumber().has_value());
  EXPECT_EQ(errors[0].LineNumber().value(), 10);
}

/////////////////////////////////////////////////
/// Check for non SDF non URDF error on a string
TEST(Parser, ReadStringErrorNotSDForURDF)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  sdf::SDFPtr sdf = InitSDF();
  const std::string testString = R"(<?xml version="1.0" ?>
    <foo>
    </foo>)";
  sdf::Errors errors;
  EXPECT_FALSE(sdf::readString(testString, sdf, errors));
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "XML does not seem to be an SDFormat or an URDF string."));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
/// Check for non SDF non URDF error on a file
TEST(Parser, ReadFileErrorNotSDForURDF)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  const auto path =
      sdf::testing::TestFile("sdf", "box.dae");

  sdf::Errors errors;
  sdf::SDFPtr sdf = sdf::readFile(path, errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARSING_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "XML does not seem to be an SDFormat or an URDF file."));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
/// Check that malformed SDF files do not throw confusing error
TEST(Parser, ReadFileMalformedSDFNoRobotError)
{
  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(false);
#endif

  const auto path =
      sdf::testing::TestFile("sdf", "bad_syntax_pose.sdf");

  sdf::Errors errors;
  sdf::SDFPtr sdf = sdf::readFile(path, errors);
  // Check the old error is not printed anymore
  EXPECT_EQ(std::string::npos, buffer.str().find(
      "Could not find the 'robot' element in the xml file"));

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
#ifdef _WIN32
  sdf::Console::Instance()->SetQuiet(true);
#endif
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  // temporarily set HOME
  std::string homeDir;
  sdf::testing::TestTmpPath(homeDir);

#ifdef _WIN32
  gz::utils::setenv("HOMEPATH", homeDir);
#else
  gz::utils::setenv("HOME", homeDir);
#endif

  sdf::Console::Clear();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
