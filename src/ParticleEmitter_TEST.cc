/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include "sdf/ParticleEmitter.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMParticleEmitter, Construction)
{
  sdf::ParticleEmitter emitter;

  EXPECT_EQ(nullptr, emitter.Element());
  EXPECT_TRUE(emitter.Name().empty());

  emitter.SetName("test_emitter");
  EXPECT_EQ(emitter.Name(), "test_emitter");

  EXPECT_EQ("point", emitter.TypeStr());
  EXPECT_EQ(sdf::ParticleEmitterType::POINT, emitter.Type());
  EXPECT_TRUE(emitter.SetType("box"));
  EXPECT_EQ("box", emitter.TypeStr());
  EXPECT_EQ(sdf::ParticleEmitterType::BOX, emitter.Type());
  emitter.SetType(sdf::ParticleEmitterType::CYLINDER);
  EXPECT_EQ("cylinder", emitter.TypeStr());

  EXPECT_TRUE(emitter.Emitting());
  emitter.SetEmitting(false);
  EXPECT_FALSE(emitter.Emitting());

  EXPECT_DOUBLE_EQ(0.0, emitter.Duration());
  emitter.SetDuration(10.0);
  EXPECT_DOUBLE_EQ(10.0, emitter.Duration());

  EXPECT_DOUBLE_EQ(5.0, emitter.Lifetime());
  emitter.SetLifetime(22.0);
  EXPECT_DOUBLE_EQ(22.0, emitter.Lifetime());
  emitter.SetLifetime(-1.0);
  EXPECT_DOUBLE_EQ(gz::math::MIN_D, emitter.Lifetime());

  EXPECT_DOUBLE_EQ(10.0, emitter.Rate());
  emitter.SetRate(123.0);
  EXPECT_DOUBLE_EQ(123.0, emitter.Rate());
  emitter.SetRate(-123.0);
  EXPECT_DOUBLE_EQ(0.0, emitter.Rate());

  EXPECT_DOUBLE_EQ(0.0, emitter.ScaleRate());
  emitter.SetScaleRate(1.2);
  EXPECT_DOUBLE_EQ(1.2, emitter.ScaleRate());
  emitter.SetScaleRate(-1.2);
  EXPECT_DOUBLE_EQ(0.0, emitter.ScaleRate());

  EXPECT_DOUBLE_EQ(1.0, emitter.MinVelocity());
  emitter.SetMinVelocity(12.4);
  EXPECT_DOUBLE_EQ(12.4, emitter.MinVelocity());
  emitter.SetMinVelocity(-12.4);
  EXPECT_DOUBLE_EQ(0.0, emitter.MinVelocity());

  EXPECT_DOUBLE_EQ(1.0, emitter.MaxVelocity());
  emitter.SetMaxVelocity(20.6);
  EXPECT_DOUBLE_EQ(20.6, emitter.MaxVelocity());
  emitter.SetMaxVelocity(-12.4);
  EXPECT_DOUBLE_EQ(0.0, emitter.MaxVelocity());

  EXPECT_EQ(gz::math::Vector3d::One, emitter.Size());
  emitter.SetSize(gz::math::Vector3d(3, 2, 1));
  EXPECT_EQ(gz::math::Vector3d(3, 2, 1), emitter.Size());
  emitter.SetSize(gz::math::Vector3d(-3, -2, -1));
  EXPECT_EQ(gz::math::Vector3d(0, 0, 0), emitter.Size());

  EXPECT_EQ(gz::math::Vector3d::One, emitter.ParticleSize());
  emitter.SetParticleSize(gz::math::Vector3d(4, 5, 6));
  EXPECT_EQ(gz::math::Vector3d(4, 5, 6), emitter.ParticleSize());
  emitter.SetParticleSize(gz::math::Vector3d(-4, -5, -6));
  EXPECT_EQ(gz::math::Vector3d(0, 0, 0), emitter.ParticleSize());

  EXPECT_EQ(gz::math::Color::White, emitter.ColorStart());
  emitter.SetColorStart(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f),
      emitter.ColorStart());

  EXPECT_EQ(gz::math::Color::White, emitter.ColorEnd());
  emitter.SetColorEnd(gz::math::Color(0.4f, 0.5f, 0.6f, 1.0f));
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 1.0f), emitter.ColorEnd());

  EXPECT_TRUE(emitter.ColorRangeImage().empty());
  emitter.SetColorRangeImage("/test/string");
  EXPECT_EQ("/test/string", emitter.ColorRangeImage());

  EXPECT_TRUE(emitter.Topic().empty());
  emitter.SetTopic("/test/topic");
  EXPECT_EQ("/test/topic", emitter.Topic());

  EXPECT_FLOAT_EQ(0.65f, emitter.ScatterRatio());
  emitter.SetScatterRatio(0.5f);
  EXPECT_FLOAT_EQ(0.5f, emitter.ScatterRatio());

  EXPECT_EQ(gz::math::Pose3d::Zero, emitter.RawPose());
  emitter.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 1.5707));
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 1.5707), emitter.RawPose());

  EXPECT_TRUE(emitter.PoseRelativeTo().empty());
  emitter.SetPoseRelativeTo("/test/relative");
  EXPECT_EQ("/test/relative", emitter.PoseRelativeTo());
}

/////////////////////////////////////////////////
TEST(DOMParticleEmitter, ToElement)
{
  sdf::ParticleEmitter emitter;

  emitter.SetName("my-emitter");
  emitter.SetType(sdf::ParticleEmitterType::BOX);
  emitter.SetEmitting(true);
  emitter.SetDuration(1.2);
  emitter.SetLifetime(3.4);
  emitter.SetRate(12.5);
  emitter.SetScaleRate(0.2);
  emitter.SetMinVelocity(32.4);
  emitter.SetMaxVelocity(50.1);
  emitter.SetSize(gz::math::Vector3d(1, 2, 3));
  emitter.SetParticleSize(gz::math::Vector3d(4, 5, 6));
  emitter.SetColorStart(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  emitter.SetColorEnd(gz::math::Color(0.4f, 0.5f, 0.6f, 1.0f));
  emitter.SetColorRangeImage("my-image");
  emitter.SetTopic("my-topic");
  emitter.SetScatterRatio(0.3f);
  emitter.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  sdf::Material material;
  emitter.SetMaterial(material);

  sdf::ElementPtr elem = emitter.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::ParticleEmitter emitter2;
  emitter2.Load(elem);

  EXPECT_EQ(emitter.Name(), emitter2.Name());
  EXPECT_EQ(emitter.Type(), emitter2.Type());
  EXPECT_EQ(emitter.Emitting(), emitter2.Emitting());
  EXPECT_DOUBLE_EQ(emitter.Duration(), emitter2.Duration());
  EXPECT_DOUBLE_EQ(emitter.Lifetime(), emitter2.Lifetime());
  EXPECT_DOUBLE_EQ(emitter.Rate(), emitter2.Rate());
  EXPECT_DOUBLE_EQ(emitter.ScaleRate(), emitter2.ScaleRate());
  EXPECT_DOUBLE_EQ(emitter.MinVelocity(), emitter2.MinVelocity());
  EXPECT_DOUBLE_EQ(emitter.MaxVelocity(), emitter2.MaxVelocity());
  EXPECT_EQ(emitter.Size(), emitter2.Size());
  EXPECT_EQ(emitter.ParticleSize(), emitter2.ParticleSize());
  EXPECT_EQ(emitter.ColorStart(), emitter2.ColorStart());
  EXPECT_EQ(emitter.ColorEnd(), emitter2.ColorEnd());
  EXPECT_EQ(emitter.ColorRangeImage(), emitter2.ColorRangeImage());
  EXPECT_EQ(emitter.Topic(), emitter2.Topic());
  EXPECT_FLOAT_EQ(emitter.ScatterRatio(), emitter2.ScatterRatio());
  EXPECT_EQ(emitter.RawPose(), emitter2.RawPose());
  EXPECT_NE(nullptr, emitter2.Material());
}

/////////////////////////////////////////////////
TEST(DOMParticleEmitter, ToElementErrorOutput)
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
  sdf::ParticleEmitter emitter;
  sdf::Errors errors;

  emitter.SetName("my-emitter");
  emitter.SetType(sdf::ParticleEmitterType::BOX);
  emitter.SetEmitting(true);
  emitter.SetDuration(1.2);
  emitter.SetLifetime(3.4);
  emitter.SetRate(12.5);
  emitter.SetScaleRate(0.2);
  emitter.SetMinVelocity(32.4);
  emitter.SetMaxVelocity(50.1);
  emitter.SetSize(gz::math::Vector3d(1, 2, 3));
  emitter.SetParticleSize(gz::math::Vector3d(4, 5, 6));
  emitter.SetColorStart(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  emitter.SetColorEnd(gz::math::Color(0.4f, 0.5f, 0.6f, 1.0f));
  emitter.SetColorRangeImage("my-image");
  emitter.SetTopic("my-topic");
  emitter.SetScatterRatio(0.3f);
  emitter.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  sdf::Material material;
  emitter.SetMaterial(material);

  sdf::ElementPtr elem = emitter.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::ParticleEmitter emitter2;
  errors = emitter2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(emitter.Name(), emitter2.Name());
  EXPECT_EQ(emitter.Type(), emitter2.Type());
  EXPECT_EQ(emitter.Emitting(), emitter2.Emitting());
  EXPECT_DOUBLE_EQ(emitter.Duration(), emitter2.Duration());
  EXPECT_DOUBLE_EQ(emitter.Lifetime(), emitter2.Lifetime());
  EXPECT_DOUBLE_EQ(emitter.Rate(), emitter2.Rate());
  EXPECT_DOUBLE_EQ(emitter.ScaleRate(), emitter2.ScaleRate());
  EXPECT_DOUBLE_EQ(emitter.MinVelocity(), emitter2.MinVelocity());
  EXPECT_DOUBLE_EQ(emitter.MaxVelocity(), emitter2.MaxVelocity());
  EXPECT_EQ(emitter.Size(), emitter2.Size());
  EXPECT_EQ(emitter.ParticleSize(), emitter2.ParticleSize());
  EXPECT_EQ(emitter.ColorStart(), emitter2.ColorStart());
  EXPECT_EQ(emitter.ColorEnd(), emitter2.ColorEnd());
  EXPECT_EQ(emitter.ColorRangeImage(), emitter2.ColorRangeImage());
  EXPECT_EQ(emitter.Topic(), emitter2.Topic());
  EXPECT_FLOAT_EQ(emitter.ScatterRatio(), emitter2.ScatterRatio());
  EXPECT_EQ(emitter.RawPose(), emitter2.RawPose());
  EXPECT_NE(nullptr, emitter2.Material());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
