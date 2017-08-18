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

#include <memory>
#include <string>

#include "sdf/system_util.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

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

    /// \brief Append the preferred path separator character for this platform
    ///        onto the passed-in string.
    /// \param[in] _s  The path to start with.
    /// \return The original path with the platform path separator appended.
    SDFORMAT_VISIBLE
    std::string const separator(std::string const &_s);

    /// \brief Append one or more additional path elements to the first
    ///        passed in argument.
    /// \param[in] args  The paths to append together
    /// \return A new string with the paths appended together.
    template<typename... Args>
    std::string append(Args const &... args)
    {
      std::string result;
      int unpack[] {
        0, (result += separator(args), 0)...};
      static_cast<void>(unpack);
      return result.substr(0, result.length() - 1);
    }

    /// \brief Get the current working path.
    /// \return Current working path if successful, the empty path on error.
    SDFORMAT_VISIBLE
    std::string current_path();

    /// \brief Given a path, get just the basename portion.
    /// \param[in] _path  The full path.
    /// \return A new string with just the basename portion of the path.
    SDFORMAT_VISIBLE
    std::string basename(const std::string &_path);

    /// \internal
    class DirIterPrivate;

    /// \class DirIter Filesystem.hh
    /// \brief A class for iterating over all items in a directory.
    class SDFORMAT_VISIBLE DirIter
    {
      /// \brief Constructor.
      /// \param[in] _in  Directory to iterate over.
      public: explicit DirIter(const std::string &_in);

      /// \brief Constructor for end element.
      public: DirIter();

      /// \brief Dereference operator; returns current directory record.
      /// \return A string representing the entire path of the directory record.
      public: std::string operator*() const;

      /// \brief Pre-increment operator; moves to next directory record.
      /// \return This iterator.
      public: const DirIter& operator++();

      /// \brief Comparison operator to see if this iterator is at the
      ///        same point as another iterator.
      /// \param[in] _other  The other iterator to compare against.
      /// \return true if the iterators are equal, false otherwise.
      public: bool operator!=(const DirIter &_other) const;

      /// \brief Destructor
      public: ~DirIter();

      /// \brief Move to the next directory record, skipping . and .. records.
      private: void next();

      /// \brief Set the internal variable to the empty string.
      private: void set_internal_empty();

      /// \brief Close an open directory handle.
      private: void close_handle();

      /// \brief Private data.
      private: std::unique_ptr<DirIterPrivate> dataPtr;
    };
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
