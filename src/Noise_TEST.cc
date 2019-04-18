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
#include "sdf/Noise.hh"

/////////////////////////////////////////////////
TEST(DOMNoise, ConstructionAndSet)
{
  sdf::Noise noise;

  EXPECT_EQ(sdf::NoiseType::NONE, noise.Type());
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  EXPECT_EQ(sdf::NoiseType::GAUSSIAN, noise.Type());

  EXPECT_DOUBLE_EQ(0.0, noise.Mean());
  noise.SetMean(1.2);
  EXPECT_DOUBLE_EQ(1.2, noise.Mean());

  EXPECT_DOUBLE_EQ(0.0, noise.StdDev());
  noise.SetStdDev(2.3);
  EXPECT_DOUBLE_EQ(2.3, noise.StdDev());

  EXPECT_DOUBLE_EQ(0.0, noise.BiasMean());
  noise.SetBiasMean(4.5);
  EXPECT_DOUBLE_EQ(4.5, noise.BiasMean());

  EXPECT_DOUBLE_EQ(0.0, noise.BiasStdDev());
  noise.SetBiasStdDev(6.7);
  EXPECT_DOUBLE_EQ(6.7, noise.BiasStdDev());

  EXPECT_DOUBLE_EQ(0.0, noise.Precision());
  noise.SetPrecision(8.9);
  EXPECT_DOUBLE_EQ(8.9, noise.Precision());

  // Copy Constructor
  sdf::Noise noise2(noise);
  EXPECT_EQ(noise, noise2);

  // Copy operator
  sdf::Noise noise3;
  noise3 = noise;
  EXPECT_EQ(noise, noise3);

  // Move Constructor
  sdf::Noise noise4(std::move(noise));
  EXPECT_EQ(noise2, noise4);

  // Move operator
  sdf::Noise noise5;
  noise5 = std::move(noise2);
  EXPECT_EQ(noise3, noise5);
}

/////////////////////////////////////////////////
TEST(DOMNoise, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  // No <noise> element
  sdf::Noise noise;
  sdf::Errors errors = noise.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <noise>")
      != std::string::npos);

  // Add the <noise> element
  sdf->SetName("noise");
  errors = noise.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("Noise is missing the type attribute")
      != std::string::npos);

  // Add the type attribute
  sdf->AddAttribute("type", "string", "none", true);
  sdf::ParamPtr param = sdf->GetAttribute("type");

  // Bad attribute value.
  param->Set("bad");
  errors = noise.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("attribute is invalid")
      != std::string::npos);

  // Good attribute value
  param->Set("gaussian");
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(sdf::NoiseType::GAUSSIAN, noise.Type())
    << "Noise type is[" << static_cast<int>(noise.Type()) << "]\n";

  // type should be case insensitive
  param->Set("GaUsSiaN_QUANTIZED");
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(sdf::NoiseType::GAUSSIAN_QUANTIZED, noise.Type())
    << "Noise type is[" << static_cast<int>(noise.Type()) << "]\n";

  // Add the <mean> child element.
  sdf::ElementPtr meanSdf(std::make_shared<sdf::Element>());
  meanSdf->SetName("mean");
  meanSdf->AddValue("double", "0.0", 0, "mean");
  meanSdf->GetValue()->Set(1.2);
  sdf->InsertElement(meanSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.Mean());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(1.2, noise.Mean());

  // Add the <stddev> child element.
  sdf::ElementPtr stdDevSdf(std::make_shared<sdf::Element>());
  stdDevSdf->SetName("stddev");
  stdDevSdf->AddValue("double", "0.0", 0, "stddev");
  stdDevSdf->GetValue()->Set(2.3);
  sdf->InsertElement(stdDevSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.StdDev());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(2.3, noise.StdDev());

  // Add the <bias_mean> child element.
  sdf::ElementPtr biasMeanSdf(std::make_shared<sdf::Element>());
  biasMeanSdf->SetName("bias_mean");
  biasMeanSdf->AddValue("double", "0.0", 0, "bias_mean");
  biasMeanSdf->GetValue()->Set(3.4);
  sdf->InsertElement(biasMeanSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.BiasMean());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(3.4, noise.BiasMean());

  // Add the <bias_stddev> child element.
  sdf::ElementPtr biasStdDevSdf(std::make_shared<sdf::Element>());
  biasStdDevSdf->SetName("bias_stddev");
  biasStdDevSdf->AddValue("double", "0.0", 0, "bias_stddev");
  biasStdDevSdf->GetValue()->Set(5.6);
  sdf->InsertElement(biasStdDevSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.BiasStdDev());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(5.6, noise.BiasStdDev());

  // Add the <precision> child element.
  sdf::ElementPtr precisionSdf(std::make_shared<sdf::Element>());
  precisionSdf->SetName("precision");
  precisionSdf->AddValue("double", "0.0", 0, "precision");
  precisionSdf->GetValue()->Set(7.8);
  sdf->InsertElement(precisionSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.Precision());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(7.8, noise.Precision());

  EXPECT_NE(nullptr, noise.Element());
  EXPECT_EQ(sdf.get(), noise.Element().get());
}
