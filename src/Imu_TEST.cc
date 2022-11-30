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
#include "sdf/Imu.hh"

/////////////////////////////////////////////////
TEST(DOMImu, Construction)
{
  sdf::Imu imu;
  sdf::Noise defaultNoise, noise;

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);

  EXPECT_EQ(defaultNoise, imu.LinearAccelerationXNoise());
  imu.SetLinearAccelerationXNoise(noise);
  EXPECT_EQ(noise, imu.LinearAccelerationXNoise());

  EXPECT_EQ(defaultNoise, imu.LinearAccelerationYNoise());
  imu.SetLinearAccelerationYNoise(noise);
  EXPECT_EQ(noise, imu.LinearAccelerationYNoise());

  EXPECT_EQ(defaultNoise, imu.LinearAccelerationZNoise());
  imu.SetLinearAccelerationZNoise(noise);
  EXPECT_EQ(noise, imu.LinearAccelerationZNoise());

  EXPECT_EQ(defaultNoise, imu.AngularVelocityXNoise());
  imu.SetAngularVelocityXNoise(noise);
  EXPECT_EQ(noise, imu.AngularVelocityXNoise());

  EXPECT_EQ(defaultNoise, imu.AngularVelocityYNoise());
  imu.SetAngularVelocityYNoise(noise);
  EXPECT_EQ(noise, imu.AngularVelocityYNoise());

  EXPECT_EQ(defaultNoise, imu.AngularVelocityZNoise());
  imu.SetAngularVelocityZNoise(noise);
  EXPECT_EQ(noise, imu.AngularVelocityZNoise());

  EXPECT_EQ(gz::math::Vector3d::UnitX, imu.GravityDirX());
  imu.SetGravityDirX(gz::math::Vector3d::Zero);
  EXPECT_EQ(gz::math::Vector3d::Zero, imu.GravityDirX());

  EXPECT_TRUE(imu.GravityDirXParentFrame().empty());
  imu.SetGravityDirXParentFrame("my_frame");
  EXPECT_EQ("my_frame", imu.GravityDirXParentFrame());

  EXPECT_EQ(gz::math::Vector3d::Zero, imu.CustomRpy());
  imu.SetCustomRpy(gz::math::Vector3d::UnitZ);
  EXPECT_EQ(gz::math::Vector3d::UnitZ, imu.CustomRpy());

  EXPECT_TRUE(imu.CustomRpyParentFrame().empty());
  imu.SetCustomRpyParentFrame("other_frame");
  EXPECT_EQ("other_frame", imu.CustomRpyParentFrame());

  EXPECT_EQ("CUSTOM", imu.Localization());
  imu.SetLocalization("NED");
  EXPECT_EQ("NED", imu.Localization());

  // Copy Constructor
  sdf::Imu imu2(imu);
  EXPECT_EQ(imu, imu2);

  // Copy operator
  sdf::Imu imu3;
  imu3 = imu;
  EXPECT_EQ(imu, imu3);

  // Move Constructor
  sdf::Imu imu4(std::move(imu));
  EXPECT_EQ(imu2, imu4);

  imu = imu4;
  EXPECT_EQ(imu2, imu);

  // Move operator
  sdf::Imu imu5;
  imu5 = std::move(imu2);
  EXPECT_EQ(imu3, imu5);

  imu2 = imu5;
  EXPECT_EQ(imu3, imu2);

  // inequality
  sdf::Imu imu6;
  EXPECT_NE(imu3, imu6);
}

/////////////////////////////////////////////////
TEST(DOMImu, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  // No <noise> element
  sdf::Imu imu;
  sdf::Errors errors = imu.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not an <imu>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, imu.Element());
  EXPECT_EQ(sdf.get(), imu.Element().get());

  // The Imu::Load function is test more thouroughly in the
  // link_dom.cc integration test.
}
