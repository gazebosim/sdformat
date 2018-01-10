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
#include "sdf/Error.hh"

/////////////////////////////////////////////////
TEST(Error, DefaultConstruction)
{
  sdf::Error error;
  EXPECT_EQ(error, false);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::NONE);
  EXPECT_TRUE(error.Message().empty());

  if (error)
    FAIL();
}

/////////////////////////////////////////////////
TEST(Error, ValueConstruction)
{
  sdf::Error error(sdf::ErrorCode::FILE_READ, "Unable to read a file");
  EXPECT_EQ(error, true);
  EXPECT_EQ(error.Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(error.Message(), "Unable to read a file");

  if (!error)
    FAIL();
}
