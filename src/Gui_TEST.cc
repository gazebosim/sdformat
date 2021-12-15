/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "sdf/Gui.hh"

/////////////////////////////////////////////////
TEST(DOMGui, Construction)
{
  sdf::Gui gui;
  EXPECT_FALSE(gui.Fullscreen());
  EXPECT_EQ(nullptr, gui.Element());
}

/////////////////////////////////////////////////
TEST(DOMGui, CopyConstruction)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::Gui gui;
  gui.Load(sdf);
  gui.SetFullscreen(true);
  EXPECT_TRUE(gui.Fullscreen());

  sdf::Gui gui2(gui);
  EXPECT_TRUE(gui2.Fullscreen());
  EXPECT_NE(nullptr, gui2.Element());
  EXPECT_EQ(gui.Element(), gui2.Element());
}

/////////////////////////////////////////////////
TEST(DOMGui, MoveConstruction)
{
  sdf::Gui gui;
  gui.SetFullscreen(true);
  EXPECT_TRUE(gui.Fullscreen());

  sdf::Gui gui2(std::move(gui));
  EXPECT_TRUE(gui2.Fullscreen());
}

/////////////////////////////////////////////////
TEST(DOMGui, MoveAssignment)
{
  sdf::Gui gui;
  gui.SetFullscreen(true);
  EXPECT_TRUE(gui.Fullscreen());

  sdf::Gui gui2;
  gui2 = std::move(gui);
  EXPECT_TRUE(gui2.Fullscreen());
}

/////////////////////////////////////////////////
TEST(DOMGui, CopyAssignment)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::Gui gui;
  gui.Load(sdf);
  gui.SetFullscreen(true);
  EXPECT_TRUE(gui.Fullscreen());

  sdf::Gui gui2;
  gui2 = gui;
  EXPECT_TRUE(gui2.Fullscreen());
  EXPECT_NE(nullptr, gui2.Element());
  EXPECT_EQ(gui.Element(), gui2.Element());
}

/////////////////////////////////////////////////
TEST(DOMGui, CopyAssignmentAfterMove)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::Gui gui1;
  gui1.Load(sdf);
  gui1.SetFullscreen(true);

  sdf::Gui gui2;
  gui2.SetFullscreen(false);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Gui tmp = std::move(gui1);
  gui1 = gui2;
  gui2 = tmp;

  EXPECT_FALSE(gui1.Fullscreen());
  EXPECT_TRUE(gui2.Fullscreen());
}

/////////////////////////////////////////////////
TEST(DOMGui, Set)
{
  sdf::Gui gui;
  gui.SetFullscreen(true);
  EXPECT_TRUE(gui.Fullscreen());
}

/////////////////////////////////////////////////
TEST(DOMGui, Equal)
{
  sdf::Gui gui;
  gui.SetFullscreen(true);
  sdf::Gui gui2(gui);

  EXPECT_TRUE(gui == gui2);
  gui.SetFullscreen(false);
  EXPECT_FALSE(gui == gui2);
}

/////////////////////////////////////////////////
TEST(DOMGui, ToElement)
{
  sdf::Gui gui;

  gui.SetFullscreen(true);

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      sdf::Plugin plugin;
      plugin.SetName("name" + std::to_string(i));
      plugin.SetFilename("filename" + std::to_string(i));
      EXPECT_TRUE(gui.AddPlugin(plugin));
      EXPECT_TRUE(gui.PluginNameExists(plugin.Name()));
      EXPECT_FALSE(gui.PluginNameExists(plugin.Name()+"a"));
      EXPECT_FALSE(gui.AddPlugin(plugin));
    }
    if (j == 0)
    {
      EXPECT_EQ(3u, gui.PluginCount());
      gui.ClearPlugins();
      EXPECT_EQ(0u, gui.PluginCount());
    }
  }


  sdf::ElementPtr elem = gui.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Gui gui2;
  gui2.Load(elem);

  EXPECT_EQ(gui.Fullscreen(), gui2.Fullscreen());
}
