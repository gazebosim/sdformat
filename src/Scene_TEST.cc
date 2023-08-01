/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "sdf/Scene.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMScene, Construction)
{
  sdf::Scene scene;
  EXPECT_EQ(gz::math::Color(0.4f, 0.4f, 0.4f), scene.Ambient());
  EXPECT_EQ(gz::math::Color(0.7f, 0.7f, 0.7f), scene.Background());
  EXPECT_TRUE(scene.Grid());
  EXPECT_TRUE(scene.Shadows());
  EXPECT_TRUE(scene.OriginVisual());
  EXPECT_EQ(nullptr, scene.Sky());
}

/////////////////////////////////////////////////
TEST(DOMScene, CopyConstruction)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::Scene scene;
  scene.Load(sdf);
  scene.SetAmbient(gz::math::Color::Blue);
  scene.SetBackground(gz::math::Color::Red);
  scene.SetGrid(false);
  scene.SetShadows(false);
  scene.SetOriginVisual(false);
  sdf::Sky sky;
  scene.SetSky(sky);

  sdf::Scene scene2(scene);
  EXPECT_EQ(gz::math::Color::Blue, scene2.Ambient());
  EXPECT_EQ(gz::math::Color::Red, scene2.Background());
  EXPECT_FALSE(scene2.Grid());
  EXPECT_FALSE(scene2.Shadows());
  EXPECT_FALSE(scene2.OriginVisual());
  EXPECT_NE(nullptr, scene2.Sky());

  EXPECT_NE(nullptr, scene2.Element());
  EXPECT_EQ(scene.Element(), scene2.Element());
}

/////////////////////////////////////////////////
TEST(DOMScene, MoveConstruction)
{
  sdf::Scene scene;
  scene.SetAmbient(gz::math::Color::Blue);
  scene.SetBackground(gz::math::Color::Red);
  scene.SetGrid(false);
  scene.SetShadows(false);
  scene.SetOriginVisual(false);
  sdf::Sky sky;
  scene.SetSky(sky);

  sdf::Scene scene2(std::move(scene));
  EXPECT_EQ(gz::math::Color::Blue, scene2.Ambient());
  EXPECT_EQ(gz::math::Color::Red, scene2.Background());
  EXPECT_FALSE(scene2.Grid());
  EXPECT_FALSE(scene2.Shadows());
  EXPECT_FALSE(scene2.OriginVisual());
  EXPECT_NE(nullptr, scene2.Sky());
}

/////////////////////////////////////////////////
TEST(DOMScene, MoveAssignmentOperator)
{
  sdf::Scene scene;
  scene.SetAmbient(gz::math::Color::Green);
  scene.SetBackground(gz::math::Color::White);
  scene.SetGrid(false);
  scene.SetShadows(false);
  scene.SetOriginVisual(false);
  sdf::Sky sky;
  scene.SetSky(sky);

  sdf::Scene scene2;
  scene2 = std::move(scene);
  EXPECT_EQ(gz::math::Color::Green, scene2.Ambient());
  EXPECT_EQ(gz::math::Color::White, scene2.Background());
  EXPECT_FALSE(scene2.Grid());
  EXPECT_FALSE(scene2.Shadows());
  EXPECT_FALSE(scene2.OriginVisual());
  EXPECT_NE(nullptr, scene2.Sky());
}

/////////////////////////////////////////////////
TEST(DOMScene, AssignmentOperator)
{
  sdf::Scene scene;
  scene.SetAmbient(gz::math::Color::Red);
  scene.SetBackground(gz::math::Color(0.2f, 0.3f, 0.4f));
  scene.SetGrid(false);
  scene.SetShadows(false);
  scene.SetOriginVisual(false);
  sdf::Sky sky;
  scene.SetSky(sky);

  sdf::Scene scene2;
  scene2 = scene;
  EXPECT_EQ(gz::math::Color::Red, scene2.Ambient());
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f), scene2.Background());
  EXPECT_FALSE(scene2.Grid());
  EXPECT_FALSE(scene2.Shadows());
  EXPECT_FALSE(scene2.OriginVisual());
  EXPECT_NE(nullptr, scene2.Sky());
}

/////////////////////////////////////////////////
TEST(DOMScene, CopyAssignmentAfterMove)
{
  sdf::Scene scene1;
  scene1.SetAmbient(gz::math::Color::Red);

  sdf::Scene scene2;
  scene2.SetAmbient(gz::math::Color::Green);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Scene tmp = std::move(scene1);
  scene1 = scene2;
  scene2 = tmp;

  EXPECT_EQ(gz::math::Color::Green, scene1.Ambient());
  EXPECT_EQ(gz::math::Color::Red, scene2.Ambient());
}

/////////////////////////////////////////////////
TEST(DOMScene, Set)
{
  sdf::Scene scene;
  scene.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f));
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f), scene.Ambient());

  scene.SetBackground(gz::math::Color(0.2f, 0.3f, 0.4f));
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f), scene.Background());

  scene.SetGrid(true);
  EXPECT_TRUE(scene.Grid());
  scene.SetGrid(false);
  EXPECT_FALSE(scene.Grid());

  scene.SetShadows(true);
  EXPECT_TRUE(scene.Shadows());
  scene.SetShadows(false);
  EXPECT_FALSE(scene.Shadows());

  scene.SetOriginVisual(true);
  EXPECT_TRUE(scene.OriginVisual());
  scene.SetOriginVisual(false);
  EXPECT_FALSE(scene.OriginVisual());

  sdf::Sky sky;
  scene.SetSky(sky);
  EXPECT_NE(nullptr, scene.Sky());
}

/////////////////////////////////////////////////
TEST(DOMScene, ToElement)
{
  sdf::Scene scene;

  scene.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  scene.SetBackground(gz::math::Color(0.2f, 0.3f, 0.4f, 1.0f));
  scene.SetGrid(true);
  scene.SetOriginVisual(true);
  scene.SetShadows(true);

  sdf::Sky sky;
  scene.SetSky(sky);

  sdf::ElementPtr elem = scene.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Scene scene2;
  scene2.Load(elem);

  EXPECT_EQ(scene.Ambient(), scene2.Ambient());
  EXPECT_EQ(scene.Background(), scene2.Background());
  EXPECT_EQ(scene.Grid(), scene2.Grid());
  EXPECT_EQ(scene.OriginVisual(), scene2.OriginVisual());
  EXPECT_EQ(scene.Shadows(), scene2.Shadows());

  const sdf::Sky *sky2 = scene2.Sky();
  ASSERT_NE(nullptr, sky2);
}

/////////////////////////////////////////////////
TEST(DOMScene, ToElementErrorOutput)
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

  sdf::Scene scene;
  sdf::Errors errors;

  scene.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  scene.SetBackground(gz::math::Color(0.2f, 0.3f, 0.4f, 1.0f));
  scene.SetGrid(true);
  scene.SetOriginVisual(true);
  scene.SetShadows(true);

  sdf::Sky sky;
  scene.SetSky(sky);

  sdf::ElementPtr elem = scene.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Scene scene2;
  errors = scene2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(scene.Ambient(), scene2.Ambient());
  EXPECT_EQ(scene.Background(), scene2.Background());
  EXPECT_EQ(scene.Grid(), scene2.Grid());
  EXPECT_EQ(scene.OriginVisual(), scene2.OriginVisual());
  EXPECT_EQ(scene.Shadows(), scene2.Shadows());

  const sdf::Sky *sky2 = scene2.Sky();
  ASSERT_NE(nullptr, sky2);

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
