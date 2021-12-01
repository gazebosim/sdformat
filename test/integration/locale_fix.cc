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

#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

// Windows supports the setlocale call but we can not extract the
// available locales using the Linux call
#ifndef _MSC_VER
TEST(CheckFixForLocal, MakeTestToFail)
{
  const std::string sdfTestFile =
      sdf::testing::TestFile("integration", "numeric.sdf");

  // Check if any of the latin locales is avilable
  FILE *fp = popen("locale -a | grep '^es\\|^pt_\\|^it_' | head -n 1", "r");

  if (!fp)
  {
    FAIL() << "locale -a call failed";
    return;
  }

  char buffer[1024];
  char *line = fgets(buffer, sizeof(buffer), fp);
  pclose(fp);

  // Do not run test if not available
  if (!line)
  {
    std::cout << "No latin locale available. Skip test" << std::endl;
    SUCCEED();
    return;
  }

  setlocale(LC_NUMERIC, line);

  // fix to allow make test without make install
  sdf::SDFPtr p(new sdf::SDF());
  sdf::init(p);
  ASSERT_TRUE(sdf::readFile(sdfTestFile, p));

  sdf::ElementPtr elem = p->Root()->GetElement("world")
    ->GetElement("physics")->GetElement("ode")->GetElement("solver")
    ->GetElement("sor");
  double angle = elem->Get<double>();
  ASSERT_DOUBLE_EQ(angle, 0.823);

  elem->Set<double>(0.423);

  // TODO(anyone): automatic checking. Error is thrown to the log file and
  // std::err We should check for "Error [Param.cc:186] Unable to set value"
  // Problem is How to get the log file path without duplicating code

  // Verify that the locale is not affecting the Param constructor
  sdf::Param param = sdf::Param("dummyDoubleParam", "double",
                                "1.5", true);
  double tmp = 0.0;
  ASSERT_TRUE(param.Get<double>(tmp));
  ASSERT_DOUBLE_EQ(1.5, tmp);
}
#endif
