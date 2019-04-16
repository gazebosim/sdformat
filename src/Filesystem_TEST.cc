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
bool create_and_switch_to_temp_dir(std::string &_new_temp_path)
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
bool create_new_file_symlink(const std::string &_symlink,
                             const std::string &_target)
{
  return symlink(_target.c_str(), _symlink.c_str()) == 0;
}

/////////////////////////////////////////////////
bool create_new_dir_symlink(const std::string &_symlink,
                            const std::string &_target)
{
  return symlink(_target.c_str(), _symlink.c_str()) == 0;
}

/////////////////////////////////////////////////
bool create_new_file_hardlink(const std::string &_hardlink,
                              const std::string &_target)
{
  return link(_target.c_str(), _hardlink.c_str()) == 0;
}

#else
#include <windows.h>
#include <winnt.h>
#include <cstdint>

/////////////////////////////////////////////////
bool create_and_switch_to_temp_dir(std::string &_new_temp_path)
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
    new_dir_name.append(std::to_string(::GetCurrentProcessId()));
    new_dir_name.push_back('_');
    // On Windows, rand_r() doesn't exist as an alternative to rand(), so the
    // cpplint warning is spurious.  This program is not multi-threaded, so
    // it is safe to suppress the threadsafe_fn warning here.
    new_dir_name.append(
       std::to_string(rand()    // NOLINT(runtime/threadsafe_fn)
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
bool create_new_file_symlink(const std::string &_symlink,
                             const std::string &_target)
{
  return ::CreateSymbolicLinkA(_symlink.c_str(), _target.c_str(), 0) == TRUE;
}

/////////////////////////////////////////////////
bool create_new_dir_symlink(const std::string &_symlink,
                            const std::string &_target)
{
  return ::CreateSymbolicLinkA(_symlink.c_str(), _target.c_str(),
                               SYMBOLIC_LINK_FLAG_DIRECTORY) == TRUE;
}

/////////////////////////////////////////////////
bool create_new_file_hardlink(const std::string &_hardlink,
                              const std::string &_target)
{
  return ::CreateHardLinkA(_hardlink.c_str(), _target.c_str(), nullptr) == TRUE;
}

#endif

#include "sdf/Filesystem.hh"

/////////////////////////////////////////////////
TEST(Filesystem, exists)
{
  std::string new_temp_dir;
  ASSERT_TRUE(create_and_switch_to_temp_dir(new_temp_dir));
  ASSERT_TRUE(create_new_empty_file("newfile"));
  ASSERT_TRUE(sdf::filesystem::create_directory("fstestexists"));

  EXPECT_TRUE(sdf::filesystem::exists("fstestexists"));
  EXPECT_TRUE(sdf::filesystem::is_directory("fstestexists"));

  EXPECT_FALSE(sdf::filesystem::exists("notcreated"));
  EXPECT_FALSE(sdf::filesystem::is_directory("notcreated"));

  EXPECT_TRUE(sdf::filesystem::exists("newfile"));
  EXPECT_FALSE(sdf::filesystem::is_directory("newfile"));
}

#ifndef _MSC_VER
/////////////////////////////////////////////////
TEST(Filesystem, symlink_exists)
{
  // There are 5 cases we want to test for links (Unix doesn't allow hard links
  // to directories or to non-existent files):
  // 1. symbolic link to existing file
  // 2. symbolic link to non-existent file
  // 3. symbolic link to existing directory
  // 4. symbolic link to non-existent directory
  // 5. hard link to existing file
  std::string new_temp_dir;
  ASSERT_TRUE(create_and_switch_to_temp_dir(new_temp_dir));
  ASSERT_TRUE(create_new_empty_file("newfile"));
  ASSERT_TRUE(sdf::filesystem::create_directory("newdir"));

  // Case 1
  ASSERT_TRUE(create_new_file_symlink("symlink-file", "newfile"));
  EXPECT_TRUE(sdf::filesystem::exists("symlink-file"));

  // Case 2
  ASSERT_TRUE(create_new_file_symlink("symlink-file-broken", "nonexistent"));
  EXPECT_FALSE(sdf::filesystem::exists("symlink-file-broken"));

  // Case 3
  ASSERT_TRUE(create_new_dir_symlink("symlink-dir", "newdir"));
  EXPECT_TRUE(sdf::filesystem::exists("symlink-dir"));

  // Case 4
  ASSERT_TRUE(create_new_dir_symlink("symlink-dir-broken", "nonexistent-dir"));
  EXPECT_FALSE(sdf::filesystem::exists("symlink-dir-broken"));

  // Case 5
  ASSERT_TRUE(create_new_file_hardlink("hardlink-file", "newfile"));
  EXPECT_TRUE(sdf::filesystem::exists("hardlink-file"));
}
#endif

/////////////////////////////////////////////////
TEST(Filesystem, current_path)
{
  std::string new_temp_dir;
  ASSERT_TRUE(create_and_switch_to_temp_dir(new_temp_dir));

  std::string path = sdf::filesystem::current_path();
  EXPECT_EQ(path, new_temp_dir);
}

/////////////////////////////////////////////////
TEST(Filesystem, append)
{
  std::string path = "tmp";
  path = sdf::filesystem::append(path, "hello");

  ASSERT_EQ(path, sdf::filesystem::separator("tmp") + "hello");

  path = sdf::filesystem::append(path, "there", "again");

  EXPECT_EQ(path, sdf::filesystem::separator("tmp") +
            sdf::filesystem::separator("hello") +
            sdf::filesystem::separator("there") + "again");
}

/////////////////////////////////////////////////
TEST(Filesystem, current_path_error)
{
  // This test intentionally creates a directory, switches to it, removes
  // the directory, and then tries to call current_path() on it to cause
  // current_path() to fail.  Windows does not allow you to remove an
  // in-use directory, so this test is restricted to Unix.
#ifndef _WIN32
  std::string new_temp_dir;
  ASSERT_TRUE(create_and_switch_to_temp_dir(new_temp_dir));

  ASSERT_EQ(rmdir(new_temp_dir.c_str()), 0);

  EXPECT_EQ(sdf::filesystem::current_path(), "");
#endif
}

/////////////////////////////////////////////////
TEST(Filesystem, basename)
{
  std::string absolute = sdf::filesystem::append("", "home", "bob", "foo");
  EXPECT_EQ(sdf::filesystem::basename(absolute), "foo");

  std::string relative = sdf::filesystem::append("baz", "foobar");
  EXPECT_EQ(sdf::filesystem::basename(relative), "foobar");

  std::string basename = "bzzz";
  EXPECT_EQ(sdf::filesystem::basename(basename), "bzzz");

  std::string nobase = sdf::filesystem::append("baz", "");
  EXPECT_EQ(sdf::filesystem::basename(nobase), "baz");

  std::string multiple_slash = sdf::filesystem::append("baz", "", "", "");
  EXPECT_EQ(sdf::filesystem::basename(multiple_slash), "baz");

  std::string multiple_slash_middle = sdf::filesystem::append("", "home", "",
                                                              "", "bob", "foo");
  EXPECT_EQ(sdf::filesystem::basename(multiple_slash_middle), "foo");

  std::string multiple_slash_start = sdf::filesystem::append("", "", "", "home",
                                                             "bob", "foo");
  EXPECT_EQ(sdf::filesystem::basename(multiple_slash_start), "foo");

  std::string slash_only = sdf::filesystem::append("", "");
  EXPECT_EQ(sdf::filesystem::basename(slash_only),
            sdf::filesystem::append("", ""));

  std::string multiple_slash_only = sdf::filesystem::append("", "", "", "");
  EXPECT_EQ(sdf::filesystem::basename(multiple_slash_only),
            sdf::filesystem::append("", ""));
}

/////////////////////////////////////////////////
TEST(Filesystem, directory_iterator)
{
  std::string new_temp_dir;
  ASSERT_TRUE(create_and_switch_to_temp_dir(new_temp_dir));
  ASSERT_TRUE(create_new_empty_file("newfile"));
  ASSERT_TRUE(sdf::filesystem::create_directory("newdir"));

#ifndef _MSC_VER
  ASSERT_TRUE(create_new_file_symlink("symlink-file", "newfile"));
  ASSERT_TRUE(create_new_file_symlink("symlink-file-broken", "nonexistent"));
  ASSERT_TRUE(create_new_dir_symlink("symlink-dir", "newdir"));
  ASSERT_TRUE(create_new_dir_symlink("symlink-dir-broken", "nonexistent-dir"));
#endif

  ASSERT_TRUE(create_new_file_hardlink("hardlink-file", "newfile"));

  std::set<std::string> found_items;

  sdf::filesystem::DirIter endIter;
  for (sdf::filesystem::DirIter dirIter("."); dirIter != endIter; ++dirIter)
  {
    found_items.insert(sdf::filesystem::basename(*dirIter));
  }

  EXPECT_FALSE(found_items.find("newfile") == found_items.end());
  EXPECT_FALSE(found_items.find("newdir") == found_items.end());

#ifndef _MSC_VER
  EXPECT_FALSE(found_items.find("symlink-file") == found_items.end());
  EXPECT_FALSE(found_items.find("symlink-file-broken") == found_items.end());
  EXPECT_FALSE(found_items.find("symlink-dir") == found_items.end());
  EXPECT_FALSE(found_items.find("symlink-dir-broken") == found_items.end());
#endif

  EXPECT_FALSE(found_items.find("hardlink-file") == found_items.end());

  found_items.clear();
  for (sdf::filesystem::DirIter dirIter(""); dirIter != endIter; ++dirIter)
  {
    found_items.insert(sdf::filesystem::basename(*dirIter));
  }

  EXPECT_EQ(found_items.size(), 0UL);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
