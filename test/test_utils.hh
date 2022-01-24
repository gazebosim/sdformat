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

/// \brief A class used for redirecting the output of sdferr, sdfwarn, etc to a
/// more convenient stream object like a std::stringstream for testing purposes.
/// The class reverts to the original stream object when it goes out of scope.
///
/// Usage example:
///
/// Redirect console output to a std::stringstream object:
///
/// ```
///   std::stringstream buffer;
///   sdf::testing::RedirectConsoleStream redir(
///       sdf::Console::Instance()->GetMsgStream(), &buffer);
///
///   sdfwarn << "Test message";
///
/// ```
/// `buffer` will now contain "Test message" with additional information,
/// such as the file and line number where sdfwarn was called from.
///
/// sdfdbg uses a log file as its output, so to redirect that, we can do
///
/// ```
///   std::stringstream buffer;
///   sdf::testing::RedirectConsoleStream redir(
///       sdf::Console::Instance()->GetLogStream(), &buffer);
///
///   sdfdbg << "Test message";
/// ```
class RedirectConsoleStream
{
  /// \brief Constructor
  /// \param[in] _consoleStream Mutable reference to a console stream.
  /// \param[in] _newSink Pointer to any object derived from std::ostream
  public: RedirectConsoleStream(sdf::Console::ConsoleStream &_consoleStream,
              std::ostream *_newSink)
      : consoleStreamRef(_consoleStream)
      , oldStream(_consoleStream)
  {
    this->consoleStreamRef = sdf::Console::ConsoleStream(_newSink);
  }

  /// \brief Destructor. Restores the console to the original ConsoleStream
  /// object.
  public: ~RedirectConsoleStream()
  {
    this->consoleStreamRef = this->oldStream;
  }

  /// \brief Reference to the console stream object. This is usually obtained
  /// from the singleton sdf::Console object by calling either
  /// sdf::Console::GetMsgStream() or sdf::Console::GetLogStream()
  private: sdf::Console::ConsoleStream &consoleStreamRef;

  /// \brief Copy of the original console stream object. This will be used to
  /// restore the console stream when this object goes out of scope.
  private: sdf::Console::ConsoleStream oldStream;
};

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

