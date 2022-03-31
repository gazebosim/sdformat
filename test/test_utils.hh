/*
 * Copyright 2021 Open Source Robotics Foundation
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
#ifndef SDF_TEST_UTILS_HH_
#define SDF_TEST_UTILS_HH_

#include <ostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include "sdf/Console.hh"
#include "sdf/Root.hh"

namespace sdf
{
namespace testing
{

/// \brief Calls a function when going out of scope.
/// Taken from:
/// https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/scope_exit.hpp
template <typename Callable>
struct ScopeExit
{
  /// \brief Constructor
  /// \param[in] _callable Any callable object that does not throw.
  explicit ScopeExit(Callable _callable)
      : callable(_callable)
  {
  }

  ~ScopeExit()
  {
    this->callable();
  }

  private: Callable callable;
};

/// \brief Initialize ignition console logging to a file. The full path to
/// the log file is returned.
/// \return Path to the console log file.
std::string InitConsoleLogFile()
{
  std::string logsDir = ignition::common::joinPaths(PROJECT_BINARY_DIR, "test",
      "test_logs");
  std::string logFilename = "console.log";
  std::string logFile = ignition::common::joinPaths(logsDir, logFilename);
  ignLogInit(logsDir, logFilename);
  return logFile;
}

/// \brief Read the contents of a log file.
/// \param[in] _file Filename to read.
/// \return Contents of the log file, or empty string if the file doesn't
/// exist.
std::string ReadConsoleLogFile(const std::string &_file)
{
  if (ignition::common::exists(_file))
  {
    std::ifstream stream(_file);
    std::string buffer((std::istreambuf_iterator<char>(stream)),
        std::istreambuf_iterator<char>());
    return buffer;
  }

  return "";
}

/// \brief Load an SDF file into a sdf::Root object
/// \param[in] _fileName The name of the file to load
/// \param[in] _root The sdf::Root object to load the file into
/// \return True if a file named _fileName was successfully loaded into
/// _root. False otherwise
bool LoadSdfFile(const std::string &_fileName, sdf::Root &_root)
{
  auto errors = _root.Load(_fileName);
  if (!errors.empty())
  {
    std::cerr << "Errors encountered:\n";
    for (const auto &e : errors)
      std::cerr << e << "\n";
    return false;
  }

  return true;
}

} // namespace testing
} // namespace sdf

#endif

