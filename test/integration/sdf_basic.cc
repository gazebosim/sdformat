/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include "sdf/sdf.hh"

#include "test_config.h"

// testing the wrapper around an sdf element WrapInRoot(const ElementPtr &_sdf)
TEST(BasicTest, RootWrapper)
{
  // create a test element
  std::string testName = "sdf-test";
  sdf::ElementPtr testElem(new sdf::Element());
  testElem->SetName(testName);
  // wrap it in the sdf root
  sdf::ElementPtr wrappedElem = sdf::SDF::WrapInRoot(testElem);
  ASSERT_NE(wrappedElem, nullptr);

  // only one top-level element expected, which is "sdf"
  EXPECT_FALSE(wrappedElem->GetNextElement());
  EXPECT_EQ(wrappedElem->GetName(), "sdf");
  // needs to have the version attribute
  EXPECT_TRUE(wrappedElem->HasAttribute("version"));
  EXPECT_EQ(wrappedElem->GetAttribute("version")->GetAsString(),
            sdf::SDF::Version());

  // check if the child element is the same test element
  sdf::ElementPtr checkElem = wrappedElem->GetFirstElement();
  ASSERT_NE(checkElem, nullptr);
  EXPECT_EQ(checkElem->GetName(), testName);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
