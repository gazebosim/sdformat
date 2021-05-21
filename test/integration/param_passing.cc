/*
 * Copyright 2020 Open Source Robotics Foundation
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
#include "sdf/Filesystem.hh"
#include "sdf/Root.hh"
#include "test_config.h"

void PrintErrors(sdf::Errors &_errors)
{
  for (sdf::Error e : _errors)
    std::cout << e.Message() << std::endl;
}

/////////////////////////////////////////////////
TEST(ParamPassingTest, ExperimentalParamsTag)
{
  const std::string modelRootPath =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "model");
  sdf::setFindCallback(
      [&](const std::string &_file)
      {
        return sdf::filesystem::append(modelRootPath, _file);
      });

  // checks normal <include> (w/o <experimental:params>)
  std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "include_model.sdf");
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  PrintErrors(errors);
  EXPECT_TRUE(errors.empty());

  // checks <include> containing <experimental:params> w/ correctly specified
  // elements
  testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "include_custom_model.sdf");
  errors = root.Load(testFile);
  PrintErrors(errors);
  EXPECT_TRUE(errors.empty());

  // compare with expected output
  testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                          "include_custom_model_expected_output.sdf");

  sdf::Root expectedRoot;
  errors = expectedRoot.Load(testFile);
  PrintErrors(errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(root.Element()->ToString(""), expectedRoot.Element()->ToString(""))
        << "ACTUAL:\n" << root.Element()->ToString("\t")
        << "\nEXPECTED:\n" << expectedRoot.Element()->ToString("\t");

  // Expected errors
  testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "include_invalid_custom_model.sdf");
  errors = root.Load(testFile);
  PrintErrors(errors);
  EXPECT_FALSE(errors.empty());
  ASSERT_EQ(errors.size(), 29u);

  int i = -1;
  // missing element_id attribute
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  // element does not exist in included model
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // element already exists in included model
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::DUPLICATE_NAME);
  // for add action, parent of specified element does not exist
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // missing name after double colons
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  // missing action attribute
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  // incorrect schema (missing required attributes; in children of element_id)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_INVALID);
  // not defined sdf element (in children of where element_id is defined)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_INVALID);
  // not defined sdf element (where element_id is defined)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_INVALID);
  // missing 'name' attribute where element_id is with action="add"
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  // 'name' attribute is empty (where element_id is with action="add")
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  // incorrect schema (missing required attributes; where element_id defined)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_INVALID);
  // invalid action
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  // invalid child action
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  // child element does not exist
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // child <box> (child of <geometry>) does not exist
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // child element (w/ name attribute) does not exist
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // child element does not exist (replace action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // missing 'name' attribute (replace action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  // invalid attribute (modify action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  // element missing (modify action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // incorrect schema (missing required attributes; where element_id defined)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_INVALID);
  // invalid attribute (modify action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  // element missing (modify action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // element missing (modify action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_MISSING);
  // element has invalid value (modify action)
  EXPECT_EQ(errors[++i].Code(), sdf::ErrorCode::ELEMENT_INVALID);
}

/////////////////////////////////////////////////
TEST(ParamPassingTest, NestedInclude)
{
  const std::string modelRootPath =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "model", "nested_include");
  sdf::setFindCallback(
      [&](const std::string &_file)
      {
        return sdf::filesystem::append(modelRootPath, _file);
      });

  // checks correctly specified elements in <experimental:params>
  // at top-level include, which has several nested includes
  // e.g., When model A includes B and B includes C. The top-level A
  //       <experimental:params> specifies elements of C
  std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration", "model",
                            "nested_include", "test_nested_include.sdf");
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  PrintErrors(errors);
  EXPECT_TRUE(errors.empty());
}
