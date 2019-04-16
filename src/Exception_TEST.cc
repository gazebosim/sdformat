/*
 * Copyright 2015 Open Source Robotics Foundation
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
#include "sdf/Exception.hh"

////////////////////////////////////////////////////
/// Test exception throwing
TEST(Exception, Throwing)
{
  EXPECT_ANY_THROW(sdfthrow("throw message"));
  EXPECT_THROW(sdfthrow("throw message"), sdf::Exception);

  EXPECT_ANY_THROW(throw sdf::Exception());
  EXPECT_THROW(throw sdf::Exception(), sdf::Exception);

  sdf::Exception ex = sdf::Exception(__FILE__, __LINE__, "testmsg\n");
  EXPECT_EQ(ex.GetErrorFile(), __FILE__);

  EXPECT_ANY_THROW(throw sdf::InternalError());
  EXPECT_THROW(throw sdf::InternalError(), sdf::InternalError);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
