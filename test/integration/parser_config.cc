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
#include "sdf/Model.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"

#include "test_config.h"

/////////////////////////////////////////////////
/// Test global config
TEST(ParserConfig, GlobalConfig)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir = PROJECT_SOURCE_PATH;

  sdf::addURIPath("file://", testDir);
  sdf::setFindCallback(
      [](const std::string &)
      {
        return "test/dir2";
      });

  const auto &config = sdf::ParserConfig::GlobalConfig();
  auto it = config.URIPathMap().find("file://");
  ASSERT_NE(config.URIPathMap().end(), it);
  ASSERT_EQ(1u, it->second.size());
  EXPECT_EQ(it->second.front(), testDir);

  ASSERT_TRUE(sdf::ParserConfig::GlobalConfig().FindFileCallback());
  EXPECT_EQ("test/dir2",
      sdf::ParserConfig::GlobalConfig().FindFileCallback()("empty"));
  // sdf::findFile requires explicitly enabling callbacks
  EXPECT_EQ("test/dir2", sdf::findFile("empty", false, true));
  EXPECT_EQ("test/dir2", sdf::findFile("empty", true, true));
}

/////////////////////////////////////////////////
/// Test using a non global config with functions like sdf::addURIPath and
/// sdf::setFindCallback
TEST(ParserConfig, NonGlobalConfig)
{
  // Reset global config
  sdf::ParserConfig::GlobalConfig() = sdf::ParserConfig();

  sdf::ParserConfig config;
  // The directory used in AddURIPath must exist in the filesystem, so we'll use
  // PROJECT_SOURCE_PATH
  const std::string testDir = PROJECT_SOURCE_PATH;
  config.AddURIPath("file://", testDir);
  config.SetFindCallback(
      [](const std::string &)
      {
        return "test/dir2";
      });

  auto it = config.URIPathMap().find("file://");
  ASSERT_NE(config.URIPathMap().end(), it);
  ASSERT_EQ(1u, it->second.size());
  EXPECT_EQ(it->second.front(), testDir);

  ASSERT_TRUE(config.FindFileCallback());
  EXPECT_EQ("test/dir2", config.FindFileCallback()("empty"));
  EXPECT_EQ("test/dir2", sdf::findFile("empty", false, true, config));
  EXPECT_EQ("test/dir2", sdf::findFile("empty", true, true, config));

  EXPECT_TRUE(sdf::ParserConfig::GlobalConfig().URIPathMap().empty());
  EXPECT_FALSE(sdf::ParserConfig::GlobalConfig().FindFileCallback());
}

/////////////////////////////////////////////////
TEST(ParserConfig, ParseWithNonGlobalConfig)
{
  // Reset global config
  sdf::ParserConfig::GlobalConfig() = sdf::ParserConfig();

  // Case 1: Use of sdf::setFindCallback
  {
    const std::string testFile =
      sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
          "includes.sdf");

    auto findFileCb = [](const std::string &_uri)
    {
      return sdf::filesystem::append(
          PROJECT_SOURCE_PATH, "test", "integration", "model", _uri);
    };

    sdf::ParserConfig config;
    config.SetFindCallback(findFileCb);

    // Parsing testFile without setting FindFileCallback on the global
    // ParserConfig should fail
    {
      sdf::Errors errors;
      sdf::SDFPtr sdf = sdf::readFile(testFile, errors);
      ASSERT_NE(nullptr, sdf);
      ASSERT_NE(nullptr, sdf->Root());
      ASSERT_TRUE(sdf->Root()->HasElement("world"));
      auto world = sdf->Root()->GetElement("world");
      EXPECT_FALSE(world->HasElement("model"));
      std::size_t uriErrorCount = std::count_if(errors.begin(), errors.end(),
          [](const auto &_err)
          {
            return _err.Code() == sdf::ErrorCode::URI_LOOKUP;
          });
      EXPECT_EQ(7u, uriErrorCount);
    }

    {
      sdf::Errors errors;
      sdf::Root root;
      errors = root.Load(testFile);
      auto world = root.WorldByIndex(0);
      ASSERT_NE(nullptr, world);
      EXPECT_EQ(0u, world->ModelCount());
      std::size_t uriErrorCount = std::count_if(errors.begin(), errors.end(),
          [](const auto &_err)
          {
            return _err.Code() == sdf::ErrorCode::URI_LOOKUP;
          });
      EXPECT_EQ(7u, uriErrorCount);
    }


    // Parsing should succeed when using a ParserConfig with the appropriate
    // findFile callback assigned
    {
      sdf::Errors errors;
      sdf::SDFPtr sdf = sdf::readFile(testFile, config, errors);
      ASSERT_NE(nullptr, sdf);
      ASSERT_NE(nullptr, sdf->Root());
      ASSERT_TRUE(sdf->Root()->HasElement("world"));
      auto world = sdf->Root()->GetElement("world");
      EXPECT_TRUE(world->HasElement("model"));
      EXPECT_TRUE(errors.empty());
    }

    {
      sdf::Errors errors;
      sdf::Root root;
      errors = root.Load(testFile, config);
      auto world = root.WorldByIndex(0);
      ASSERT_NE(nullptr, world);
      EXPECT_EQ(3u, world->ModelCount());
      EXPECT_TRUE(errors.empty());
    }
  }

  // Case 2: Use of sdf::addURIPath
  {
    const std::string testSdfString = R"(
    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>testScheme://box</uri> <!-- NOLINT -->
        </include>
      </world>
    </sdf>)";

    sdf::ParserConfig config;
    config.AddURIPath("testScheme://",
        sdf::filesystem::append(
            PROJECT_SOURCE_PATH, "test", "integration", "model"));

    // Parsing testSdfString without setting addURIPath on the global
    // ParserConfig should fail
    {
      sdf::Errors errors;
      sdf::SDFPtr sdf = std::make_shared<sdf::SDF>();
      sdf::init(sdf);
      sdf::readString(testSdfString, sdf, errors);
      ASSERT_NE(nullptr, sdf);
      ASSERT_NE(nullptr, sdf->Root());
      ASSERT_TRUE(sdf->Root()->HasElement("world"));
      auto world = sdf->Root()->GetElement("world");
      EXPECT_FALSE(world->HasElement("model"));
      std::size_t uriErrorCount = std::count_if(errors.begin(), errors.end(),
          [](const auto &_err)
          {
            return _err.Code() == sdf::ErrorCode::URI_LOOKUP;
          });
      EXPECT_EQ(1u, uriErrorCount);
    }
    {
      sdf::Errors errors;
      sdf::Root root;
      errors = root.LoadSdfString(testSdfString);
      auto world = root.WorldByIndex(0);
      ASSERT_NE(nullptr, world);
      EXPECT_EQ(0u, world->ModelCount());
      std::size_t uriErrorCount = std::count_if(errors.begin(), errors.end(),
          [](const auto &_err)
          {
            return _err.Code() == sdf::ErrorCode::URI_LOOKUP;
          });
      EXPECT_EQ(1u, uriErrorCount);
    }

    // Parsing should succeed when using a ParserConfig with the appropriate
    // uri map
    {
      sdf::Errors errors;
      sdf::SDFPtr sdf = std::make_shared<sdf::SDF>();
      sdf::init(sdf);
      sdf::readString(testSdfString, config, sdf, errors);
      ASSERT_NE(nullptr, sdf);
      ASSERT_NE(nullptr, sdf->Root());
      ASSERT_TRUE(sdf->Root()->HasElement("world"));
      auto world = sdf->Root()->GetElement("world");
      EXPECT_TRUE(world->HasElement("model"));
      EXPECT_TRUE(errors.empty());
    }
    {
      sdf::Errors errors;
      sdf::Root root;
      errors = root.LoadSdfString(testSdfString, config);
      auto world = root.WorldByIndex(0);
      ASSERT_NE(nullptr, world);
      EXPECT_EQ(1u, world->ModelCount());
      EXPECT_TRUE(errors.empty());
    }
  }
}
