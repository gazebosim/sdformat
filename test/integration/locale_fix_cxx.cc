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

#include <locale>

#include <gtest/gtest.h>

#include "sdf/Param.hh"

TEST(CheckFixForLocal, CheckFixForCxxLocal)
{
  struct CommaDecimalPointFacet : std::numpunct<char>
  {
    char do_decimal_point() const
    {
      return ',';
    }
  };

  // Set a global locale in which the decimal separator is the comma
  std::locale newLocale(std::locale::classic(), new CommaDecimalPointFacet);
  std::locale originalGlobalLocale = std::locale::global(newLocale);

  // Create param with vector2d default value
  sdf::Param param = sdf::Param("dummyVec2DParam", "vector2d",
                                "1.5 2.5", true);

  // Verify that the default value is correctly parsed
  gz::math::Vector2d vectmp;
  ASSERT_TRUE(param.Get<gz::math::Vector2d>(vectmp));
  ASSERT_DOUBLE_EQ(1.5, vectmp[0]);
  ASSERT_DOUBLE_EQ(2.5, vectmp[1]);

  // Restore the original global locale
  std::locale prevLocale = std::locale::global(originalGlobalLocale);
  EXPECT_EQ(newLocale, prevLocale);
}
