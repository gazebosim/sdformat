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
#include "sdf/sdf_config.h"
#include "sdf/Root.hh"

/////////////////////////////////////////////////
TEST(DOMRoot, Construction)
{
  sdf::Root root;
  EXPECT_EQ(root.Version(), "");
  EXPECT_FALSE(root.WorldNameExists("default"));
  EXPECT_FALSE(root.WorldNameExists(""));
  EXPECT_EQ(root.WorldCount(), 0u);
  EXPECT_TRUE(root.WorldByIndex(0) == nullptr);
  EXPECT_TRUE(root.WorldByIndex(1) == nullptr);
}

/////////////////////////////////////////////////
TEST(DOMRoot, Set)
{
  sdf::Root root;
  EXPECT_STREQ(root.Version().c_str(), "");
  root.SetVersion(SDF_PROTOCOL_VERSION);
  EXPECT_STREQ(root.Version().c_str(), SDF_PROTOCOL_VERSION);
}
