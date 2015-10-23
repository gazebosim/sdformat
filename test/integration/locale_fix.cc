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
#include "sdf/sdf.hh"
#include "sdf/parser_urdf.hh"

#include "test_config.h"

const std::string SDF_TEST_FILE = std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/numeric.sdf";

// Windows supports the setlocale call but we can not extract the
// available locales using the Linux call
#ifndef _MSC_VER
TEST(CheckFixForLocal, MakeTestToFail)
{
  // Check if any of the latin locales is avilable
  FILE *fp = popen("locale -a | grep '^es\\|^pt_\\|^it_' | head -n 1", "r");

  if (!fp)
    FAIL() << "locale -a call failed";

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
  ASSERT_TRUE(sdf::readFile(SDF_TEST_FILE, p));

  sdf::ElementPtr elem = p->Root()->GetElement("world")
    ->GetElement("physics")->GetElement("ode")->GetElement("solver")
    ->GetElement("sor");
  double angle = elem->Get<double>();
  ASSERT_DOUBLE_EQ(angle, 0.823);

  elem->Set<double>(0.423);

  // TODO: automatic checking. Error is thrown to the log file and std::err
  // We should check for "Error [Param.cc:186] Unable to set value"
  // Problem is How to get the log file path without duplicating code
}
#endif
