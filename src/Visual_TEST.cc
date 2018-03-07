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
#include "sdf/Visual.hh"

/////////////////////////////////////////////////
TEST(DOMVisual, Construction)
{
  sdf::Visual visual;
  EXPECT_TRUE(visual.Name().empty());

  visual.SetName("test_visual");
  EXPECT_EQ(visual.Name(), "test_visual");

  EXPECT_EQ(ignition::math::Pose3d::Zero, visual.Pose());
  EXPECT_TRUE(visual.PoseFrame().empty());

  visual.SetPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.Pose());

  visual.SetPoseFrame("link");
  EXPECT_EQ("link", visual.PoseFrame());
}
