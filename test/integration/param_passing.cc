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

void print_errors(sdf::Errors &_errors)
{
  for (sdf::Error e : _errors)
    std::cout << e.Message() << std::endl;
}

/////////////////////////////////////////////////
TEST(ParamPassingTest, ExperimentalParamsTag)
{
  const std::string modelRootPath =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "model") + "/";
  sdf::setFindCallback(
      [&](const std::string &_file)
      {
        return modelRootPath + _file;
      });

  std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "include_model.sdf");
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  print_errors(errors);

  EXPECT_TRUE(errors.empty());

  testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "include_custom_model.sdf");
  errors = root.Load(testFile);
  print_errors(errors);

  EXPECT_TRUE(errors.empty());

  testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "include_invalid_custom_model.sdf");
  errors = root.Load(testFile);
  print_errors(errors);

  EXPECT_FALSE(errors.empty());
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::ELEMENT_MISSING);
}
