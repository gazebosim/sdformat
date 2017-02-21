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

#ifndef _SDF_FILESYSTEM_HH_
#define _SDF_FILESYSTEM_HH_

#include <string>

#include "sdf/system_util.hh"

namespace sdf
{
  namespace filesystem
  {
    /// \brief Determine whether the given path exists on the filesystem.
    /// \param[in] _path  The path to check for existence
    /// \return True if the path exists on the filesystem, false otherwise.
    SDFORMAT_VISIBLE
    bool exists(const std::string &_path);

    /// \brief Determine whether the given path is a directory.
    /// \param[in] _path  The path to check
    /// \return True if given path exists and is a directory, false otherwise.
    SDFORMAT_VISIBLE
    bool is_directory(const std::string &_path);

    /// \brief Create a new directory on the filesystem.  Intermediate
    ///        directories must already exist.
    /// \param[in] _path  The new directory path to create
    /// \return True if directory creation was successful, false otherwise.
    SDFORMAT_VISIBLE
    bool create_directory(const std::string &_path);

    // The below is C++ variadic template magic to allow an append
    // method that takes 1-n number of arguments to append together.
    SDFORMAT_VISIBLE
    std::string const separator(std::string const &_s);

    /// \brief Append one or more additional path elements to the first
    ///        passed in argument.
    /// \param[in] args  The paths to append together
    /// \return A new string with the paths appended together.
    template<typename... Args>
    SDFORMAT_VISIBLE
    std::string append(Args const &... args)
    {
      std::string result;
      int unpack[] {
        0, (result += separator(args), 0)...};
      static_cast<void>(unpack);
      return result.substr(0, result.length() - 1);
    }
  }
}

#endif
