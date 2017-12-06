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
#include "sdf/dom/Light.hh"

/////////////////////////////////////////////////
TEST(DOMLight, Construction)
{
  sdf::Light light;

  EXPECT_EQ(light.Type(), sdf::LightType::UNKNOWN);
  EXPECT_EQ(light.Typename(), "unknown");
}

/////////////////////////////////////////////////
TEST(DOMLight, Set)
{
  sdf::Light light;

  EXPECT_EQ(light.Type(), sdf::LightType::UNKNOWN);
  EXPECT_EQ(light.Typename(), "unknown");

  light.SetType(sdf::LightType::DIRECTIONAL);
  EXPECT_EQ(light.Type(), sdf::LightType::DIRECTIONAL);
  EXPECT_EQ(light.Typename(), "directional");

  light.SetType(sdf::LightType::POINT);
  EXPECT_EQ(light.Type(), sdf::LightType::POINT);
  EXPECT_EQ(light.Typename(), "point");

  light.SetType(sdf::LightType::SPOT);
  EXPECT_EQ(light.Type(), sdf::LightType::POINT);
  EXPECT_EQ(light.Typename(), "point");
}
