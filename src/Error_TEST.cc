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
#include "sdf/config.hh"
#include "sdf/Exception.hh"
#include "sdf/Error.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(Error, DefaultConstruction)
{
  sdf::Error error;
  EXPECT_EQ(error, false);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::NONE);
  EXPECT_TRUE(error.Message().empty());
  EXPECT_FALSE(error.XmlPath().has_value());
  EXPECT_FALSE(error.FilePath().has_value());
  EXPECT_FALSE(error.LineNumber().has_value());

  if (error)
  {
    FAIL();
  }
  error.SetXmlPath("/sdf/world");
  ASSERT_TRUE(error.XmlPath().has_value());
  EXPECT_EQ("/sdf/world", error.XmlPath());

  error.SetFilePath("/tmp/test_file.sdf");
  ASSERT_TRUE(error.FilePath().has_value());
  EXPECT_EQ("/tmp/test_file.sdf", error.FilePath());

  error.SetLineNumber(5);
  ASSERT_TRUE(error.LineNumber().has_value());
  EXPECT_EQ(5, error.LineNumber());
}

/////////////////////////////////////////////////
TEST(Error, ValueConstructionWithoutFile)
{
  sdf::Error error(sdf::ErrorCode::FILE_READ, "Unable to read a file");
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");
  EXPECT_FALSE(error.XmlPath().has_value());
  EXPECT_FALSE(error.FilePath().has_value());
  EXPECT_FALSE(error.LineNumber().has_value());

  if (!error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ValueConstructionWithFile)
{
  const std::string emptyFilePath = "Empty/file/path";
  sdf::Error error(
    sdf::ErrorCode::FILE_READ,
    "Unable to read a file",
    emptyFilePath);
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");
  EXPECT_FALSE(error.XmlPath().has_value());
  ASSERT_TRUE(error.FilePath().has_value());
  EXPECT_EQ(error.FilePath().value(), emptyFilePath);
  EXPECT_FALSE(error.LineNumber().has_value());

  if (!error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ValueConstructionWithLineNumber)
{
  const std::string emptyFilePath = "Empty/file/path";
  const int lineNumber = 10;
  sdf::Error error(
    sdf::ErrorCode::FILE_READ,
    "Unable to read a file",
    emptyFilePath,
    lineNumber);
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");
  EXPECT_FALSE(error.XmlPath().has_value());
  ASSERT_TRUE(error.FilePath().has_value());
  EXPECT_EQ(error.FilePath().value(), emptyFilePath);
  ASSERT_TRUE(error.LineNumber().has_value());
  EXPECT_EQ(error.LineNumber().value(), lineNumber);

  if (!error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ValueConstructionWithXmlPath)
{
  const std::string emptyFilePath = "Empty/file/path";
  const int lineNumber = 10;
  sdf::Error error(
    sdf::ErrorCode::FILE_READ,
    "Unable to read a file",
    emptyFilePath,
    lineNumber);
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");
  ASSERT_TRUE(error.FilePath().has_value());
  EXPECT_EQ(error.FilePath().value(), emptyFilePath);
  ASSERT_TRUE(error.LineNumber().has_value());
  EXPECT_EQ(error.LineNumber().value(), lineNumber);

  const std::string emptyXmlPath = "/sdf/model";
  ASSERT_FALSE(error.XmlPath().has_value());
  error.SetXmlPath(emptyXmlPath);
  ASSERT_TRUE(error.XmlPath().has_value());
  EXPECT_EQ(error.XmlPath().value(), emptyXmlPath);

  if (!error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ThrowOrPrint)
{
  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif
  {
    sdf::Error error(sdf::ErrorCode::DUPLICATE_NAME, "Duplicate found");
    std::stringstream buffer;
    sdf::testing::RedirectConsoleStream redir(
        sdf::Console::Instance()->GetMsgStream(), &buffer);

    sdf::internal::throwOrPrintError(sdferr, error);
    EXPECT_NE(std::string::npos, buffer.str().find("Duplicate found"))
        << buffer.str();
  }

  {
    std::stringstream buffer;
    sdf::testing::RedirectConsoleStream redir(
        sdf::Console::Instance()->GetMsgStream(), &buffer);
    sdf::Error error(sdf::ErrorCode::FATAL_ERROR, "Fatal Error");
    EXPECT_THROW(sdf::internal::throwOrPrintError(sdferr, error),
                 sdf::AssertionInternalError);
  }
}
