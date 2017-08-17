/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <string>

#include <gtest/gtest.h>

#ifndef _WIN32
#include <stdio.h>
#include <stdlib.h>
#endif

#include "sdf/Console.hh"

#ifndef _WIN32
bool create_new_temp_dir(std::string &_new_temp_path)
{
  std::string tmppath;
  const char *tmp = getenv("TMPDIR");
  if (tmp)
  {
    tmppath = std::string(tmp);
  }
  else
  {
    tmppath = std::string("/tmp");
  }

  tmppath += "/XXXXXX";

  char *dtemp = mkdtemp(const_cast<char *>(tmppath.c_str()));
  if (dtemp == nullptr)
  {
    return false;
  }

  _new_temp_path = std::string(dtemp);

  return true;
}

////////////////////////////////////////////////////
TEST(Console, nohome)
{
  sdf::Console::Clear();
  unsetenv("HOME");

  sdferr << "Error.\n";
}

////////////////////////////////////////////////////
TEST(Console, logdir_is_file)
{
  sdf::Console::Clear();

  std::string temp_dir;
  ASSERT_TRUE(create_new_temp_dir(temp_dir));
  ASSERT_EQ(setenv("HOME", temp_dir.c_str(), 1), 0);

  std::string sdf_file = temp_dir + "/.sdformat";

  FILE *fp = fopen(sdf_file.c_str(), "w");
  ASSERT_NE(fp, nullptr);
  ASSERT_EQ(fwrite("hello", 5, 1, fp), 1UL);
  fclose(fp);

  sdferr << "Error.\n";
}

////////////////////////////////////////////////////
TEST(Console, logdir_doesnt_exist)
{
  sdf::Console::Clear();

  std::string temp_dir;
  ASSERT_TRUE(create_new_temp_dir(temp_dir));
  temp_dir += "/foobarbaz";
  ASSERT_EQ(setenv("HOME", temp_dir.c_str(), 1), 0);

  sdferr << "Error.\n";
}

#endif  // _WIN32

////////////////////////////////////////////////////
/// Test out the different console messages.
TEST(Console, Messages)
{
  sdferr << "Error.\n";
  sdfwarn << "Warning.\n";
  sdfmsg << "Message.\n";
  sdfdbg << "Debug.\n";
}

////////////////////////////////////////////////////
TEST(Console, quiet)
{
  sdf::ConsolePtr con = sdf::Console::Instance();

  con->SetQuiet(true);

  sdferr << "Error.\n";
  sdfwarn << "Warning.\n";
  sdfmsg << "Message.\n";
  sdfdbg << "Debug.\n";

  con->SetQuiet(false);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
