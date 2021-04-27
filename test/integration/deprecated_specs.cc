/*
 * Copyright 2018 Open Source Robotics Foundation
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

////////////////////////////////////////////////////
TEST(DeprecatedSpecs, Spec1_0)
{
  const std::string filename =
    sdf::testing::TestFile("integration", "deprecated_sdf_1-0.sdf");
  sdf::SDFPtr sdf(new sdf::SDF());
  EXPECT_FALSE(sdf::initFile(filename, sdf));
}

////////////////////////////////////////////////////
TEST(DeprecatedSpecs, Spec1_2)
{
  const std::string filename =
    sdf::testing::TestFile("integration", "deprecated_sdf_1-2.sdf");
  sdf::SDFPtr sdf(new sdf::SDF());
  EXPECT_FALSE(sdf::initFile(filename, sdf));
}
