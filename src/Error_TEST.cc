/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <optional>
#include "sdf/sdf_config.h"
#include "sdf/Error.hh"

/////////////////////////////////////////////////
TEST(Error, DefaultConstruction)
{
  sdf::Error error;
  EXPECT_EQ(error, false);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::NONE);
  EXPECT_TRUE(error.Message().empty());
  EXPECT_FALSE(error.FilePath().has_value());
  EXPECT_FALSE(error.LineNumber().has_value());

  if (error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ValueConstructionWithoutFile)
{
  sdf::Error error(sdf::ErrorCode::FILE_READ, "Unable to read a file");
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");
  EXPECT_FALSE(error.FilePath().has_value());
  EXPECT_FALSE(error.LineNumber().has_value());

  if (!error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ValueConstructionWithFile)
{
  const std::string emptyFilePath = "Empty/file/path";
  sdf::Error error(sdf::ErrorCode::FILE_READ, "Unable to read a file",
                   emptyFilePath, 10);
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");
  EXPECT_TRUE(error.FilePath().has_value());
  EXPECT_EQ(error.FilePath().value(), emptyFilePath);
  EXPECT_TRUE(error.LineNumber().has_value());
  EXPECT_EQ(error.LineNumber().value(), 10);

  if (!error)
    FAIL();
}
