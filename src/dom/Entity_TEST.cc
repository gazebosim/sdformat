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
#include "sdf/dom/Entity.hh"

namespace sdf
{
  // Entity is pure virtual class. This class stubs out the pure virtual
  // functions.
  class MockEntity : public Entity
  {
    public: bool Load(sdf::ElementPtr /*_sdf*/)
    {
      return true;
    }
  };
}

/////////////////////////////////////////////////
TEST(DOMEntity, Construction)
{
  sdf::MockEntity entity;
  EXPECT_TRUE(entity.Name().empty());
  EXPECT_TRUE(entity.Frame().empty());
  EXPECT_EQ(entity.Pose(), ignition::math::Pose3d::Zero);
}

/////////////////////////////////////////////////
TEST(DOMEntity, Set)
{
  sdf::MockEntity entity;
  EXPECT_TRUE(entity.Name().empty());
  EXPECT_TRUE(entity.Frame().empty());
  EXPECT_EQ(entity.Pose(), ignition::math::Pose3d::Zero);

  entity.SetName("testname");
  entity.SetFrame("testframe");
  entity.SetPose(ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_EQ(entity.Name(), "testname");
  EXPECT_EQ(entity.Frame(), "testframe");
  EXPECT_EQ(entity.Pose(), ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
}
