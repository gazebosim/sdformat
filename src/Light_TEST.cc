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
#include <ignition/math/Pose3.hh>
#include "sdf/Light.hh"

/////////////////////////////////////////////////
TEST(DOMLight, DefaultConstruction)
{
  sdf::Light light;
  EXPECT_EQ(nullptr, light.Element());
  EXPECT_EQ(sdf::LightType::POINT, light.Type());
  EXPECT_TRUE(light.Name().empty());

  light.SetName("test_light");
  EXPECT_EQ("test_light", light.Name());

  EXPECT_EQ(ignition::math::Pose3d::Zero, light.Pose());
  EXPECT_TRUE(light.PoseFrame().empty());

  light.SetPose({1, 2, 3, 0, 0, IGN_PI});
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, IGN_PI), light.Pose());

  light.SetPoseFrame("world");
  EXPECT_EQ("world", light.PoseFrame());

  EXPECT_FALSE(light.CastShadows());
  light.SetCastShadows(true);
  EXPECT_TRUE(light.CastShadows());

  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), light.Diffuse());
  light.SetDiffuse(ignition::math::Color(0.1, 0.2, 0.3, 1.0));
  EXPECT_EQ(ignition::math::Color(0.1, 0.2, 0.3, 1), light.Diffuse());

  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), light.Specular());
  light.SetSpecular(ignition::math::Color(0.4, 0.6, 0.7, 1.0));
  EXPECT_EQ(ignition::math::Color(0.4, 0.6, 0.7, 1), light.Specular());

  EXPECT_DOUBLE_EQ(10.0, light.AttenuationRange());
  light.SetAttenuationRange(1.2);
  EXPECT_DOUBLE_EQ(1.2, light.AttenuationRange());

  EXPECT_DOUBLE_EQ(1.0, light.LinearAttenuationFactor());
  light.SetLinearAttenuationFactor(0.2);
  EXPECT_DOUBLE_EQ(0.2, light.LinearAttenuationFactor());

  EXPECT_DOUBLE_EQ(1.0, light.ConstantAttenuationFactor());
  light.SetConstantAttenuationFactor(0.4);
  EXPECT_DOUBLE_EQ(0.4, light.ConstantAttenuationFactor());

  EXPECT_DOUBLE_EQ(0.0, light.QuadraticAttenuationFactor());
  light.SetQuadraticAttenuationFactor(1.1);
  EXPECT_DOUBLE_EQ(1.1, light.QuadraticAttenuationFactor());

  EXPECT_EQ(ignition::math::Vector3d(0, 0, -1), light.Direction());
  light.SetDirection({0.4, 0.2, 0});
  EXPECT_EQ(ignition::math::Vector3d(0.4, 0.2, 0), light.Direction());

  EXPECT_EQ(ignition::math::Angle(0.0), light.SpotInnerAngle());
  light.SetSpotInnerAngle(1.4);
  EXPECT_EQ(ignition::math::Angle(1.4), light.SpotInnerAngle());

  EXPECT_EQ(ignition::math::Angle(0.0), light.SpotOuterAngle());
  light.SetSpotOuterAngle(0.2);
  EXPECT_EQ(ignition::math::Angle(0.2), light.SpotOuterAngle());

  EXPECT_DOUBLE_EQ(0.0, light.SpotFalloff());
  light.SetSpotFalloff(4.3);
  EXPECT_DOUBLE_EQ(4.3, light.SpotFalloff());
}

/////////////////////////////////////////////////
TEST(DOMLight, SpotLightNegativeValues)
{
  sdf::Light light;
  light.SetSpotFalloff(-1.0);
  EXPECT_DOUBLE_EQ(0.0, light.SpotFalloff());

  light.SetSpotInnerAngle({-1.0});
  EXPECT_DOUBLE_EQ(0.0, light.SpotInnerAngle().Radian());

  light.SetSpotOuterAngle({-2.0});
  EXPECT_DOUBLE_EQ(0.0, light.SpotOuterAngle().Radian());
}

/////////////////////////////////////////////////
TEST(DOMLight, AttenuationClamp)
{
  sdf::Light light;

  light.SetLinearAttenuationFactor(-1.0);
  EXPECT_DOUBLE_EQ(0.0, light.LinearAttenuationFactor());

  light.SetLinearAttenuationFactor(20.0);
  EXPECT_DOUBLE_EQ(1.0, light.LinearAttenuationFactor());

  light.SetConstantAttenuationFactor(-1.0);
  EXPECT_DOUBLE_EQ(0.0, light.ConstantAttenuationFactor());

  light.SetConstantAttenuationFactor(20.0);
  EXPECT_DOUBLE_EQ(1.0, light.ConstantAttenuationFactor());

  light.SetQuadraticAttenuationFactor(-1.0);
  EXPECT_DOUBLE_EQ(0.0, light.QuadraticAttenuationFactor());
}
