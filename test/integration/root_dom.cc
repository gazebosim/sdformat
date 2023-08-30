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

#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "test_config.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMRoot, InvalidSDF)
{
  // Redirect sdferr output
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

  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty_invalid.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_EQ(errors.size(), 2u);
  EXPECT_EQ(sdf::ErrorCode::PARSING_ERROR, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "XML does not seem to be an SDFormat or an URDF file."));
  EXPECT_EQ(sdf::ErrorCode::FILE_READ, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "Unable to read file"));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMRoot, NoVersion)
{
    // Redirect sdferr output
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

  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty_noversion.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::FILE_READ, errors[0].Code());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMRoot, Load)
{
  // Redirect sdferr output
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

  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty.sdf");

  sdf::Root root;
  EXPECT_EQ(0u, root.WorldCount());
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_EQ(SDF_PROTOCOL_VERSION, root.Version());
  EXPECT_EQ(1u, root.WorldCount());
  EXPECT_TRUE(root.WorldByIndex(0) != nullptr);
  EXPECT_TRUE(root.WorldByIndex(1) == nullptr);

  EXPECT_EQ("default", root.WorldByIndex(0)->Name());

  EXPECT_EQ(1u, root.WorldByIndex(0)->ModelCount());
  ASSERT_TRUE(root.WorldByIndex(0)->ModelByIndex(0) != nullptr);
  EXPECT_EQ("ground_plane", root.WorldByIndex(0)->ModelByIndex(0)->Name());
  EXPECT_TRUE(root.WorldByIndex(0)->ModelNameExists("ground_plane"));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadMultipleModels)
{
  // Redirect sdferr output
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

  const std::string testFile =
    sdf::testing::TestFile("sdf", "root_multiple_models.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);

  // An error should be emitted since there are multiple root models.
  // errors. For now, only the first model is loaded.
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_EQ("Root object can only contain one model. Using the first one found",
            errors[0].Message());

  ASSERT_NE(nullptr, root.Model());
  EXPECT_EQ("robot1", root.Model()->Name());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadDuplicateModels)
{
  // Redirect sdferr output
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
  const std::string testFile =
    sdf::testing::TestFile("sdf", "root_duplicate_models.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::DUPLICATE_NAME, errors[0].Code());
  EXPECT_EQ("model with name[robot1] already exists.", errors[0].Message());

  EXPECT_NE(nullptr, root.Model());
  EXPECT_EQ("robot1", root.Model()->Name());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMRoot, CreateMulipleWorlds)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf",
        "world_frame_attached_to.sdf");

  // Load the SDF file
  sdf::Root loadedRoot;
  EXPECT_TRUE(loadedRoot.Load(testFile).empty());
  sdf::World *loadedWorld = loadedRoot.WorldByIndex(0);

  sdf::Root root;
  loadedWorld->SetName("world0");
  EXPECT_TRUE(root.AddWorld(*loadedWorld).empty());

  loadedWorld->SetName("world1");
  EXPECT_TRUE(root.AddWorld(*loadedWorld).empty());
  EXPECT_FALSE(root.AddWorld(*loadedWorld).empty());

  auto testFunc = std::function<void(sdf::Root &)>(
      [](sdf::Root &_root)
  {
    EXPECT_EQ(2u, _root.WorldCount());
    for (int i = 0; i < 1; ++i)
    {
      // Get the first world
      const sdf::World *world = _root.WorldByIndex(i);
      ASSERT_NE(nullptr, world);

      EXPECT_EQ(std::string("world") + std::to_string(i), world->Name());
      EXPECT_EQ(1u, world->ModelCount());
      EXPECT_NE(nullptr, world->ModelByIndex(0));
      EXPECT_EQ(nullptr, world->ModelByIndex(1));

      EXPECT_TRUE(world->ModelNameExists("M1"));

      const sdf::Model *model = world->ModelByIndex(0);
      ASSERT_NE(nullptr, model);
      EXPECT_EQ("M1", model->Name());
      EXPECT_EQ(1u, model->LinkCount());
      EXPECT_NE(nullptr, model->LinkByIndex(0));
      EXPECT_EQ(nullptr, model->LinkByIndex(1));
      EXPECT_EQ(1u, model->FrameCount());
      EXPECT_NE(nullptr, model->FrameByIndex(0));
      EXPECT_EQ(nullptr, model->FrameByIndex(1));
      ASSERT_TRUE(model->LinkNameExists("L"));
      ASSERT_TRUE(model->FrameNameExists("F0"));
      EXPECT_EQ("L", model->FrameByName("F0")->AttachedTo());

      EXPECT_EQ(5u, world->FrameCount());
      EXPECT_NE(nullptr, world->FrameByIndex(0));
      EXPECT_NE(nullptr, world->FrameByIndex(1));
      EXPECT_NE(nullptr, world->FrameByIndex(2));
      EXPECT_NE(nullptr, world->FrameByIndex(3));
      EXPECT_NE(nullptr, world->FrameByIndex(4));
      EXPECT_EQ(nullptr, world->FrameByIndex(5));
      ASSERT_TRUE(world->FrameNameExists("world_frame"));
      ASSERT_TRUE(world->FrameNameExists("F00"));
      ASSERT_TRUE(world->FrameNameExists("F0"));
      ASSERT_TRUE(world->FrameNameExists("F1"));
      ASSERT_TRUE(world->FrameNameExists("F2"));

      EXPECT_TRUE(world->FrameByName("world_frame")->AttachedTo().empty());
      EXPECT_TRUE(world->FrameByName("F0")->AttachedTo().empty());
      EXPECT_EQ("world", world->FrameByName("F00")->AttachedTo());
      EXPECT_EQ("F0", world->FrameByName("F1")->AttachedTo());
      EXPECT_EQ("M1", world->FrameByName("F2")->AttachedTo());

      EXPECT_TRUE(world->FrameByName("world_frame")->PoseRelativeTo().empty());
      EXPECT_TRUE(world->FrameByName("F00")->PoseRelativeTo().empty());
      EXPECT_TRUE(world->FrameByName("F0")->PoseRelativeTo().empty());
      EXPECT_TRUE(world->FrameByName("F1")->PoseRelativeTo().empty());
      EXPECT_TRUE(world->FrameByName("F2")->PoseRelativeTo().empty());

      std::string body;
      EXPECT_TRUE(
        world->FrameByName("world_frame")->ResolveAttachedToBody(body).empty());
      EXPECT_EQ("world", body);
      EXPECT_TRUE(
          world->FrameByName("F00")->ResolveAttachedToBody(body).empty());
      EXPECT_EQ("world", body);
      EXPECT_TRUE(
          world->FrameByName("F0")->ResolveAttachedToBody(body).empty());
      EXPECT_EQ("world", body);
      EXPECT_TRUE(
          world->FrameByName("F1")->ResolveAttachedToBody(body).empty());
      EXPECT_EQ("world", body);
      EXPECT_TRUE(
          world->FrameByName("F2")->ResolveAttachedToBody(body).empty());
      EXPECT_EQ("M1::L", body);
    }
  });

  testFunc(root);

  // Test UpdateGraphs
  EXPECT_TRUE(root.UpdateGraphs().empty());
  testFunc(root);

  // Test cloning
  sdf::Root root2 = root.Clone();
  testFunc(root);
  testFunc(root2);
}
