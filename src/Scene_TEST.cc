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

/////////////////////////////////////////////////
TEST(DOMScene, Construction)
{
  sdf::Scene scene;
  EXPECT_EQ(ignition::math::Color::Black, scene.Ambient());
  EXPECT_EQ(ignition::math::Color::Black, scene.Background());
  EXPECT_FALSE(scene.Grid());
  EXPECT_FALSE(scene.Shadows());
  EXPECT_FALSE(scene.OriginVisual());
}

/////////////////////////////////////////////////
TEST(DOMScene, CopyConstruction)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::Scene scene;
  scene.Load(sdf);
  scene.SetAmbient(ignition::math::Color::Blue);
  scene.SetBackground(ignition::math::Color::Red);
  scene.SetGrid(true);
  scene.SetShadows(true);
  scene.SetOriginVisual(true);

  sdf::Scene scene2(scene);
  EXPECT_EQ(ignition::math::Color::Blue, scene2.Ambient());
  EXPECT_EQ(ignition::math::Color::Red, scene2.Background());
  EXPECT_TRUE(scene2.Grid());
  EXPECT_TRUE(scene2.Shadows());
  EXPECT_TRUE(scene2.OriginVisual());

  EXPECT_NE(nullptr, scene2.Element());
  EXPECT_EQ(scene.Element(), scene2.Element());
}

/////////////////////////////////////////////////
TEST(DOMScene, MoveConstruction)
{
  sdf::Scene scene;
  scene.SetAmbient(ignition::math::Color::Blue);
  scene.SetBackground(ignition::math::Color::Red);
  scene.SetGrid(true);
  scene.SetShadows(true);
  scene.SetOriginVisual(true);

  sdf::Scene scene2(std::move(scene));
  EXPECT_EQ(ignition::math::Color::Blue, scene2.Ambient());
  EXPECT_EQ(ignition::math::Color::Red, scene2.Background());
  EXPECT_TRUE(scene2.Grid());
  EXPECT_TRUE(scene2.Shadows());
  EXPECT_TRUE(scene2.OriginVisual());
}

/////////////////////////////////////////////////
TEST(DOMScene, MoveAssignmentOperator)
{
  sdf::Scene scene;
  scene.SetAmbient(ignition::math::Color::Green);
  scene.SetBackground(ignition::math::Color::White);
  scene.SetGrid(true);
  scene.SetShadows(true);
  scene.SetOriginVisual(true);

  sdf::Scene scene2 = std::move(scene);
  EXPECT_EQ(ignition::math::Color::Green, scene2.Ambient());
  EXPECT_EQ(ignition::math::Color::White, scene2.Background());
  EXPECT_TRUE(scene2.Grid());
  EXPECT_TRUE(scene2.Shadows());
  EXPECT_TRUE(scene2.OriginVisual());
}

/////////////////////////////////////////////////
TEST(DOMScene, AssignmentOperator)
{
  sdf::Scene scene;
  scene.SetAmbient(ignition::math::Color::Red);
  scene.SetBackground(ignition::math::Color(0.2, 0.3, 0.4));
  scene.SetGrid(true);
  scene.SetShadows(true);
  scene.SetOriginVisual(true);

  sdf::Scene scene2 = scene;
  EXPECT_EQ(ignition::math::Color::Red, scene2.Ambient());
  EXPECT_EQ(ignition::math::Color(0.2, 0.3, 0.4), scene2.Background());
  EXPECT_TRUE(scene2.Grid());
  EXPECT_TRUE(scene2.Shadows());
  EXPECT_TRUE(scene2.OriginVisual());
}

/////////////////////////////////////////////////
TEST(DOMScene, Set)
{
  sdf::Scene scene;
  scene.SetAmbient(ignition::math::Color(0.1, 0.2, 0.3));
  EXPECT_EQ(ignition::math::Color(0.1, 0.2, 0.3), scene.Ambient());

  scene.SetBackground(ignition::math::Color(0.1, 0.2, 0.3));
  EXPECT_EQ(ignition::math::Color(0.1, 0.2, 0.3), scene.Ambient());

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
}
