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

  EXPECT_DOUBLE_EQ(0.0, noise.DynamicBiasStdDev());
  noise.SetDynamicBiasStdDev(9.1);
  EXPECT_DOUBLE_EQ(9.1, noise.DynamicBiasStdDev());

  EXPECT_DOUBLE_EQ(0.0, noise.DynamicBiasCorrelationTime());
  noise.SetDynamicBiasCorrelationTime(19.12);
  EXPECT_DOUBLE_EQ(19.12, noise.DynamicBiasCorrelationTime());

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


  // Copy assignment after move
  sdf::Noise noise6;
  noise6.SetMean(0.6);
  sdf::Noise noise7;
  noise7.SetMean(0.7);

  sdf::Noise tmp = std::move(noise6);
  noise6 = noise7;
  noise7 = tmp;
  EXPECT_DOUBLE_EQ(0.7, noise6.Mean());
  EXPECT_DOUBLE_EQ(0.6, noise7.Mean());
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
  errors.clear();
  sdf->AddAttribute("type", "string", "none", true, errors);
  ASSERT_TRUE(errors.empty());
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
  meanSdf->AddValue("double", "0.0", 0, errors, "mean");
  EXPECT_TRUE(errors.empty());
  meanSdf->GetValue()->Set(1.2);
  sdf->InsertElement(meanSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.Mean());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(1.2, noise.Mean());

  // Add the <stddev> child element.
  sdf::ElementPtr stdDevSdf(std::make_shared<sdf::Element>());
  stdDevSdf->SetName("stddev");
  stdDevSdf->AddValue("double", "0.0", 0, errors,"stddev");
  EXPECT_TRUE(errors.empty());
  stdDevSdf->GetValue()->Set(2.3);
  sdf->InsertElement(stdDevSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.StdDev());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(2.3, noise.StdDev());

  // Add the <bias_mean> child element.
  sdf::ElementPtr biasMeanSdf(std::make_shared<sdf::Element>());
  biasMeanSdf->SetName("bias_mean");
  biasMeanSdf->AddValue("double", "0.0", 0, errors, "bias_mean");
  EXPECT_TRUE(errors.empty());
  biasMeanSdf->GetValue()->Set(3.4);
  sdf->InsertElement(biasMeanSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.BiasMean());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(3.4, noise.BiasMean());

  // Add the <bias_stddev> child element.
  sdf::ElementPtr biasStdDevSdf(std::make_shared<sdf::Element>());
  biasStdDevSdf->SetName("bias_stddev");
  biasStdDevSdf->AddValue("double", "0.0", 0, errors, "bias_stddev");
  EXPECT_TRUE(errors.empty());
  biasStdDevSdf->GetValue()->Set(5.6);
  sdf->InsertElement(biasStdDevSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.BiasStdDev());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(5.6, noise.BiasStdDev());

  // Add the <precision> child element.
  sdf::ElementPtr precisionSdf(std::make_shared<sdf::Element>());
  precisionSdf->SetName("precision");
  precisionSdf->AddValue("double", "0.0", 0, errors, "precision");
  EXPECT_TRUE(errors.empty());
  precisionSdf->GetValue()->Set(7.8);
  sdf->InsertElement(precisionSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.Precision());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(7.8, noise.Precision());

  EXPECT_NE(nullptr, noise.Element());
  EXPECT_EQ(sdf.get(), noise.Element().get());

  // Add the <dynamic_bias_stddev> child element.
  sdf::ElementPtr dynamicBiasStdDevSdf(std::make_shared<sdf::Element>());
  dynamicBiasStdDevSdf->SetName("dynamic_bias_stddev");
  dynamicBiasStdDevSdf->AddValue("double", "0.0", 0, errors,
      "dynamic_bias_stddev");
  EXPECT_TRUE(errors.empty());
  dynamicBiasStdDevSdf->GetValue()->Set(8.9);
  sdf->InsertElement(dynamicBiasStdDevSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.DynamicBiasStdDev());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(8.9, noise.DynamicBiasStdDev());

  // Add the <dynamic_bias_correlation_time> child element.
  sdf::ElementPtr dynamicBiasCTimeSdf(std::make_shared<sdf::Element>());
  dynamicBiasCTimeSdf->SetName("dynamic_bias_correlation_time");
  dynamicBiasCTimeSdf->AddValue("double", "0.0", 0, errors,
      "dynamic_bias_correlation_time");
  EXPECT_TRUE(errors.empty());
  dynamicBiasCTimeSdf->GetValue()->Set(10.2);
  sdf->InsertElement(dynamicBiasCTimeSdf);

  EXPECT_DOUBLE_EQ(0.0, noise.DynamicBiasCorrelationTime());
  errors = noise.Load(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(10.2, noise.DynamicBiasCorrelationTime());
}

/////////////////////////////////////////////////
TEST(DOMNoise, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  noise.SetDynamicBiasStdDev(9.1);
  noise.SetDynamicBiasCorrelationTime(19.12);

  sdf::Errors errors;
  sdf::ElementPtr noiseElem = noise.ToElement(errors);
  ASSERT_TRUE(errors.empty());
  EXPECT_NE(nullptr, noiseElem);
  EXPECT_EQ(nullptr, noise.Element());

  // verify values after loading the element back
  sdf::Noise noise2;
  noise2.Load(noiseElem);

  EXPECT_EQ(sdf::NoiseType::GAUSSIAN, noise2.Type());
  EXPECT_DOUBLE_EQ(1.2, noise2.Mean());
  EXPECT_DOUBLE_EQ(2.3, noise2.StdDev());
  EXPECT_DOUBLE_EQ(4.5, noise2.BiasMean());
  EXPECT_DOUBLE_EQ(6.7, noise2.BiasStdDev());
  EXPECT_DOUBLE_EQ(8.9, noise2.Precision());
  EXPECT_DOUBLE_EQ(9.1, noise2.DynamicBiasStdDev());
  EXPECT_DOUBLE_EQ(19.12, noise2.DynamicBiasCorrelationTime());

  // make changes to DOM and verify ToElement produces updated values
  noise2.SetPrecision(0.1234);
  sdf::ElementPtr noise2Elem = noise2.ToElement(errors);
  ASSERT_TRUE(errors.empty());
  EXPECT_NE(nullptr, noise2Elem);
  sdf::Noise noise3;
  noise3.Load(noise2Elem);
  EXPECT_DOUBLE_EQ(0.1234, noise3.Precision());
}
