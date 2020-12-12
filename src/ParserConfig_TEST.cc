/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "sdf/Filesystem.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/SDFImpl.hh"
#include "test_config.h"

/////////////////////////////////////////////////
/// Test default construction of sdf::ParserConfig.
TEST(ParserConfig, Construction)
{
  sdf::ParserConfig config;
  EXPECT_FALSE(config.FindFileCallback());
  EXPECT_TRUE(config.URIPathMap().empty());

  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir = PROJECT_SOURCE_PATH;

  config.AddURIPath("file://", testDir);
  ASSERT_FALSE(config.URIPathMap().empty());
  {
    auto it = config.URIPathMap().find("file://");
    ASSERT_NE(config.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir);
  }

  auto testFunc = [](const std::string &)
  {
    return "test/dir2";
  };

  config.SetFindCallback(testFunc);
  ASSERT_TRUE(config.FindFileCallback());
  EXPECT_EQ("test/dir2", config.FindFileCallback()("empty"));
}

/////////////////////////////////////////////////
TEST(ParserConfig, CopyConstructor)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir1 = PROJECT_SOURCE_PATH;
  const std::string testDir2 =
      sdf::filesystem::append(PROJECT_SOURCE_PATH, "test");

  sdf::ParserConfig config1;
  config1.AddURIPath("file://", testDir1);

  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }

  sdf::ParserConfig config2(config1);
  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }

  config2.AddURIPath("file://", testDir2);
  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(2u, it->second.size());
    EXPECT_EQ(it->second.back(), testDir2);
  }

  // Updating config2 should not affect config1
  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }
}

/////////////////////////////////////////////////
TEST(ParserConfig, CopyAssignmentOperator)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir1 = PROJECT_SOURCE_PATH;
  const std::string testDir2 =
      sdf::filesystem::append(PROJECT_SOURCE_PATH, "test");

  sdf::ParserConfig config1;
  config1.AddURIPath("file://", testDir1);
  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }

  sdf::ParserConfig config2;
  config2 = config1;
  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }

  config2.AddURIPath("file://", testDir2);
  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(2u, it->second.size());
    EXPECT_EQ(it->second.back(), testDir2);
  }

  // Updating config2 should not affect config1
  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }
}

/////////////////////////////////////////////////
TEST(ParserConfig, MoveConstructor)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir1 = PROJECT_SOURCE_PATH;

  sdf::ParserConfig config1;
  config1.AddURIPath("file://", testDir1);

  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }

  sdf::ParserConfig config2(std::move(config1));
  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }
}

/////////////////////////////////////////////////
TEST(ParserConfig, MoveAssignmentOperator)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir1 = PROJECT_SOURCE_PATH;

  sdf::ParserConfig config1;
  config1.AddURIPath("file://", testDir1);

  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }

  sdf::ParserConfig config2;
  config2 = std::move(config1);
  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }
}

/////////////////////////////////////////////////
TEST(ParserConfig, CopyAssignmentAfterMove)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir1 = PROJECT_SOURCE_PATH;
  const std::string testDir2 =
      sdf::filesystem::append(PROJECT_SOURCE_PATH, "test");

  sdf::ParserConfig config1;
  config1.AddURIPath("file://", testDir1);

  sdf::ParserConfig config2;
  config2.AddURIPath("file://", testDir2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::ParserConfig tmp = std::move(config1);
  config1 = config2;
  config2 = tmp;

  {
    auto it = config1.URIPathMap().find("file://");
    ASSERT_NE(config1.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir2);
  }

  {
    auto it = config2.URIPathMap().find("file://");
    ASSERT_NE(config2.URIPathMap().end(), it);
    ASSERT_EQ(1u, it->second.size());
    EXPECT_EQ(it->second.front(), testDir1);
  }
}
