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
#include "test_utils.hh"

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

  EXPECT_TRUE(imu.OrientationEnabled());
  imu.SetOrientationEnabled(false);
  EXPECT_FALSE(imu.OrientationEnabled());

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

  // The Imu::Load function is test more thoroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMImu, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Imu imu;
  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  imu.SetLinearAccelerationXNoise(noise);
  imu.SetLinearAccelerationYNoise(noise);
  imu.SetLinearAccelerationZNoise(noise);
  imu.SetAngularVelocityXNoise(noise);
  imu.SetAngularVelocityYNoise(noise);
  imu.SetAngularVelocityZNoise(noise);
  imu.SetGravityDirX(gz::math::Vector3d::Zero);
  imu.SetGravityDirXParentFrame("my_frame");
  imu.SetCustomRpy(gz::math::Vector3d::UnitZ);
  imu.SetCustomRpyParentFrame("other_frame");
  imu.SetLocalization("NED");
  imu.SetOrientationEnabled(false);

  sdf::ElementPtr imuElem = imu.ToElement();
  EXPECT_NE(nullptr, imuElem);
  EXPECT_EQ(nullptr, imu.Element());

  // verify values after loading the element back
  sdf::Imu imu2;
  imu2.Load(imuElem);

  EXPECT_EQ(noise, imu2.LinearAccelerationXNoise());
  EXPECT_EQ(noise, imu2.LinearAccelerationYNoise());
  EXPECT_EQ(noise, imu2.LinearAccelerationZNoise());
  EXPECT_EQ(noise, imu2.AngularVelocityXNoise());
  EXPECT_EQ(noise, imu2.AngularVelocityYNoise());
  EXPECT_EQ(noise, imu2.AngularVelocityZNoise());
  EXPECT_EQ(gz::math::Vector3d::Zero, imu2.GravityDirX());
  EXPECT_EQ("my_frame", imu2.GravityDirXParentFrame());
  EXPECT_EQ(gz::math::Vector3d::UnitZ, imu2.CustomRpy());
  EXPECT_EQ("other_frame", imu2.CustomRpyParentFrame());
  EXPECT_EQ("NED", imu2.Localization());
  EXPECT_FALSE(imu2.OrientationEnabled());

  // make changes to DOM and verify ToElement produces updated values
  imu2.SetGravityDirX(gz::math::Vector3d(1, 2, 3));;
  sdf::ElementPtr imu2Elem = imu2.ToElement();
  EXPECT_NE(nullptr, imu2Elem);
  sdf::Imu imu3;
  imu3.Load(imu2Elem);
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), imu3.GravityDirX());
}

/////////////////////////////////////////////////
TEST(DOMImu, ToElementErrorOutput)
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

  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Imu imu;
  sdf::Noise noise;
  sdf::Errors errors;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  imu.SetLinearAccelerationXNoise(noise);
  imu.SetLinearAccelerationYNoise(noise);
  imu.SetLinearAccelerationZNoise(noise);
  imu.SetAngularVelocityXNoise(noise);
  imu.SetAngularVelocityYNoise(noise);
  imu.SetAngularVelocityZNoise(noise);
  imu.SetGravityDirX(gz::math::Vector3d::Zero);
  imu.SetGravityDirXParentFrame("my_frame");
  imu.SetCustomRpy(gz::math::Vector3d::UnitZ);
  imu.SetCustomRpyParentFrame("other_frame");
  imu.SetLocalization("NED");
  imu.SetOrientationEnabled(false);

  sdf::ElementPtr imuElem = imu.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, imuElem);
  EXPECT_EQ(nullptr, imu.Element());

  // verify values after loading the element back
  sdf::Imu imu2;
  errors = imu2.Load(imuElem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(noise, imu2.LinearAccelerationXNoise());
  EXPECT_EQ(noise, imu2.LinearAccelerationYNoise());
  EXPECT_EQ(noise, imu2.LinearAccelerationZNoise());
  EXPECT_EQ(noise, imu2.AngularVelocityXNoise());
  EXPECT_EQ(noise, imu2.AngularVelocityYNoise());
  EXPECT_EQ(noise, imu2.AngularVelocityZNoise());
  EXPECT_EQ(gz::math::Vector3d::Zero, imu2.GravityDirX());
  EXPECT_EQ("my_frame", imu2.GravityDirXParentFrame());
  EXPECT_EQ(gz::math::Vector3d::UnitZ, imu2.CustomRpy());
  EXPECT_EQ("other_frame", imu2.CustomRpyParentFrame());
  EXPECT_EQ("NED", imu2.Localization());
  EXPECT_FALSE(imu2.OrientationEnabled());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
