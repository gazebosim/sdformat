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
#include "sdf/ForceTorque.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMForceTorque, Construction)
{
  sdf::ForceTorque ft;
  sdf::Noise defaultNoise, noise;

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);

  EXPECT_EQ(defaultNoise, ft.ForceXNoise());
  ft.SetForceXNoise(noise);
  EXPECT_EQ(noise, ft.ForceXNoise());

  EXPECT_EQ(defaultNoise, ft.ForceYNoise());
  ft.SetForceYNoise(noise);
  EXPECT_EQ(noise, ft.ForceYNoise());

  EXPECT_EQ(defaultNoise, ft.ForceZNoise());
  ft.SetForceZNoise(noise);
  EXPECT_EQ(noise, ft.ForceZNoise());

  EXPECT_EQ(defaultNoise, ft.TorqueXNoise());
  ft.SetTorqueXNoise(noise);
  EXPECT_EQ(noise, ft.TorqueXNoise());

  EXPECT_EQ(defaultNoise, ft.TorqueYNoise());
  ft.SetTorqueYNoise(noise);
  EXPECT_EQ(noise, ft.TorqueYNoise());

  EXPECT_EQ(defaultNoise, ft.TorqueZNoise());
  ft.SetTorqueZNoise(noise);
  EXPECT_EQ(noise, ft.TorqueZNoise());

  EXPECT_EQ(ft.Frame(), sdf::ForceTorqueFrame::CHILD);
  ft.SetFrame(sdf::ForceTorqueFrame::PARENT);
  EXPECT_EQ(ft.Frame(), sdf::ForceTorqueFrame::PARENT);

  EXPECT_EQ(ft.MeasureDirection(),
            sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT);
  ft.SetMeasureDirection(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD);
  EXPECT_EQ(ft.MeasureDirection(),
            sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD);

  // Copy constructor
  sdf::ForceTorque ft2(ft);
  EXPECT_EQ(ft2, ft);

  // Copy operator
  sdf::ForceTorque ft3;
  ft3 = ft;
  EXPECT_EQ(ft3, ft);

  // Move constructor
  sdf::ForceTorque ft4(std::move(ft));
  EXPECT_EQ(ft4, ft2);
  ft = ft4;
  EXPECT_EQ(ft, ft2);

  // Move operator
  sdf::ForceTorque ft5;
  ft5 = std::move(ft2);
  EXPECT_EQ(ft5, ft3);
  ft2 = ft5;
  EXPECT_EQ(ft2, ft3);

  // Inequality
  sdf::ForceTorque ft6;
  EXPECT_NE(ft6, ft3);
}

/////////////////////////////////////////////////
TEST(DOMForceTorque, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::ForceTorque ft;
  sdf::Errors errors = ft.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <force_torque>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, ft.Element());
  EXPECT_EQ(sdf.get(), ft.Element().get());

  // The ForceTorque::Load function is tested more thoroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMForceTorque, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::ForceTorque ft;
  sdf::Noise noise;

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  ft.SetForceXNoise(noise);
  ft.SetForceYNoise(noise);
  ft.SetForceZNoise(noise);
  ft.SetTorqueXNoise(noise);
  ft.SetTorqueYNoise(noise);
  ft.SetTorqueZNoise(noise);
  ft.SetFrame(sdf::ForceTorqueFrame::PARENT);
  ft.SetMeasureDirection(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD);

  sdf::ElementPtr ftElem = ft.ToElement();
  EXPECT_NE(nullptr, ftElem);
  EXPECT_EQ(nullptr, ft.Element());

  // verify values after loading the element back
  sdf::ForceTorque ft2;
  ft2.Load(ftElem);

  EXPECT_EQ(noise, ft2.ForceXNoise());
  EXPECT_EQ(noise, ft2.ForceYNoise());
  EXPECT_EQ(noise, ft2.ForceZNoise());
  EXPECT_EQ(noise, ft2.TorqueXNoise());
  EXPECT_EQ(noise, ft2.TorqueYNoise());
  EXPECT_EQ(noise, ft2.TorqueZNoise());
  EXPECT_EQ(sdf::ForceTorqueFrame::PARENT, ft2.Frame());
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD,
      ft2.MeasureDirection());

  // make changes to DOM and verify ToElement produces updated values
  ft2.SetMeasureDirection(sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT);
  sdf::ElementPtr ft2Elem = ft2.ToElement();
  EXPECT_NE(nullptr, ft2Elem);
  sdf::ForceTorque ft3;
  ft3.Load(ft2Elem);
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT,
      ft3.MeasureDirection());
}

/////////////////////////////////////////////////
TEST(DOMForceTorque, ToElementErrorOutput)
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
  sdf::ForceTorque ft;
  sdf::Noise noise;
  sdf::Errors errors;

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  ft.SetForceXNoise(noise);
  ft.SetForceYNoise(noise);
  ft.SetForceZNoise(noise);
  ft.SetTorqueXNoise(noise);
  ft.SetTorqueYNoise(noise);
  ft.SetTorqueZNoise(noise);
  ft.SetFrame(sdf::ForceTorqueFrame::PARENT);
  ft.SetMeasureDirection(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD);

  sdf::ElementPtr ftElem = ft.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, ftElem);
  EXPECT_EQ(nullptr, ft.Element());

  // verify values after loading the element back
  sdf::ForceTorque ft2;
  errors = ft2.Load(ftElem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(noise, ft2.ForceXNoise());
  EXPECT_EQ(noise, ft2.ForceYNoise());
  EXPECT_EQ(noise, ft2.ForceZNoise());
  EXPECT_EQ(noise, ft2.TorqueXNoise());
  EXPECT_EQ(noise, ft2.TorqueYNoise());
  EXPECT_EQ(noise, ft2.TorqueZNoise());
  EXPECT_EQ(sdf::ForceTorqueFrame::PARENT, ft2.Frame());
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD,
      ft2.MeasureDirection());

  // make changes to DOM and verify ToElement produces updated values
  ft2.SetMeasureDirection(sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT);
  sdf::ElementPtr ft2Elem = ft2.ToElement();
  EXPECT_NE(nullptr, ft2Elem);
  sdf::ForceTorque ft3;
  ft3.Load(ft2Elem);
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT,
      ft3.MeasureDirection());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
