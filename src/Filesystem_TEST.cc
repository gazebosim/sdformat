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

#ifndef _WIN32
#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/////////////////////////////////////////////////
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
  if (chdir(dtemp) < 0)
  {
    return false;
  }

  char resolved[PATH_MAX];
  if (realpath(dtemp, resolved) == nullptr)
  {
    return false;
  }

  _new_temp_path = std::string(resolved);

  return true;
}

/////////////////////////////////////////////////
bool create_new_empty_file(const std::string &_filename)
{
  int fd = open(_filename.c_str(), O_RDWR | O_CREAT, 0644);
  if (fd < 0)
  {
    return false;
  }

  close(fd);

  return true;
}

/////////////////////////////////////////////////
std::string preferred_separator()
{
  return "/";
}
#else
#include <windows.h>
#include <winnt.h>
#include <cstdint>

/////////////////////////////////////////////////
std::string int_to_string(int _value)
{
  const int kOutputBufSize = 3 * sizeof(_value) + 1;

  std::string outbuf(kOutputBufSize, 0);
  bool is_neg = _value < 0;
  unsigned int res = static_cast<unsigned int>(_value < 0 ? -_value : _value);

  for (std::string::iterator it = outbuf.end();;)
  {
    --it;
    *it = static_cast<std::string::value_type>((res % 10) + '0');
    res /= 10;
    if (res == 0)
    {
      if (is_neg)
      {
        --it;
        *it = static_cast<std::string::value_type>('-');
      }
      return std::string(it, outbuf.end());
    }
  }
}

/////////////////////////////////////////////////
bool create_new_temp_dir(std::string &_new_temp_path)
{
  char temp_path[MAX_PATH + 1];
  DWORD path_len = ::GetTempPathA(MAX_PATH, temp_path);
  if (path_len >= MAX_PATH || path_len <= 0)
  {
    return false;
  }
  std::string path_to_create(temp_path);
  srand(static_cast<uint32_t>(time(nullptr)));

  for (int count = 0; count < 50; ++count)
  {
    // Try creating a new temporary directory with a randomly generated name.
    // If the one we chose exists, keep trying another path name until we reach
    // some limit.
    std::string new_dir_name;
    new_dir_name.append(int_to_string(::GetCurrentProcessId()));
    new_dir_name.push_back('_');
    // On Windows, rand_r() doesn't exist as an alternative to rand(), so the
    // cpplint warning is spurious.  This program is not multi-threaded, so
    // it is safe to suppress the threadsafe_fn warning here.
    new_dir_name.append(
       int_to_string(rand()    // NOLINT(runtime/threadsafe_fn)
                     % ((int16_t)0x7fff)));

    path_to_create += new_dir_name;
    if (::CreateDirectoryA(path_to_create.c_str(), nullptr))
    {
      _new_temp_path = path_to_create;
      return ::SetCurrentDirectoryA(_new_temp_path.c_str()) != 0;
    }
  }

  return false;
}

/////////////////////////////////////////////////
bool create_new_empty_file(const std::string &_filename)
{
  return ::CreateFileA(_filename.c_str(),
                       FILE_READ_DATA,
                       FILE_SHARE_READ,
                       nullptr,
                       OPEN_ALWAYS,
                       0,
                       nullptr) != INVALID_HANDLE_VALUE;
}

/////////////////////////////////////////////////
std::string preferred_separator()
{
  return "\\";
}
#endif

#include "sdf/Filesystem.hh"

/////////////////////////////////////////////////
TEST(Filesystem, exists)
{
  std::string new_temp_dir;
  create_new_temp_dir(new_temp_dir);
  ASSERT_TRUE(create_new_empty_file("newfile"));

  sdf::filesystem::create_directory("fstestexists");
  EXPECT_TRUE(sdf::filesystem::exists("fstestexists"));
  EXPECT_TRUE(sdf::filesystem::is_directory("fstestexists"));

  EXPECT_FALSE(sdf::filesystem::exists("notcreated"));
  EXPECT_FALSE(sdf::filesystem::is_directory("notcreated"));

  EXPECT_TRUE(sdf::filesystem::exists("newfile"));
  EXPECT_FALSE(sdf::filesystem::is_directory("newfile"));
}

/////////////////////////////////////////////////
TEST(Filesystem, current_path)
{
  std::string new_temp_dir;
  create_new_temp_dir(new_temp_dir);

  std::string path = sdf::filesystem::current_path();
  EXPECT_EQ(path, new_temp_dir);
}

/////////////////////////////////////////////////
TEST(Filesystem, append)
{
  std::string path = "tmp";
  path = sdf::filesystem::append(path, "hello");

  ASSERT_EQ(path, "tmp" + preferred_separator() + "hello");

  path = sdf::filesystem::append(path, "there", "again");

  ASSERT_EQ(path, "tmp" + preferred_separator() +
            "hello" + preferred_separator() +
            "there" + preferred_separator() + "again");
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
