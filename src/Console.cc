/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "sdf/Console.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Types.hh"

using namespace sdf;

/// Static pointer to the console.
static std::shared_ptr<Console> myself;
static std::mutex g_instance_mutex;

/// \todo Output disabled for windows, to allow tests to pass. We should
/// disable output just for tests on windows.
#ifndef _WIN32
static bool g_quiet = false;
#else
static bool g_quiet = true;
#endif

static Console::ConsoleStream g_NullStream(nullptr);

//////////////////////////////////////////////////
Console::Console()
  : dataPtr(new ConsolePrivate)
{
  // Set up the file that we'll log to.
#ifndef _WIN32
  const char *home = std::getenv("HOME");
#else
  const char *home = std::getenv("HOMEPATH");
#endif
  if (!home)
  {
    std::cerr << "No HOME defined in the environment. Will not log."
              << std::endl;
    return;
  }
  std::string logDir = sdf::filesystem::append(home, ".sdformat");
  if (!sdf::filesystem::exists(logDir))
  {
    sdf::filesystem::create_directory(logDir);
  }
  else if (!sdf::filesystem::is_directory(logDir))
  {
    std::cerr << logDir << " exists but is not a directory.  Will not log."
              << std::endl;
    return;
  }
  std::string logFile = sdf::filesystem::append(logDir, "sdformat.log");
  this->dataPtr->logFileStream.open(logFile.c_str(), std::ios::out);
}

//////////////////////////////////////////////////
Console::~Console()
{
}

//////////////////////////////////////////////////
ConsolePtr Console::Instance()
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  if (!myself)
  {
    myself.reset(new Console());
  }

  return myself;
}

//////////////////////////////////////////////////
void Console::Clear()
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);

  myself = nullptr;
}

//////////////////////////////////////////////////
void Console::SetQuiet(bool _quiet)
{
  g_quiet = _quiet;
}

//////////////////////////////////////////////////
Console::ConsoleStream &Console::ColorMsg(const std::string &lbl,
                                          const std::string &file,
                                          unsigned int line, int color)
{
  if (!g_quiet)
  {
    this->dataPtr->msgStream.Prefix(lbl, file, line, color);
    return this->dataPtr->msgStream;
  }
  else
  {
    return g_NullStream;
  }
}

//////////////////////////////////////////////////
Console::ConsoleStream &Console::Log(const std::string &lbl,
                                     const std::string &file,
                                     unsigned int line)
{
  this->dataPtr->logStream.Prefix(lbl, file, line, 0);
  return this->dataPtr->logStream;
}

//////////////////////////////////////////////////
void Console::ConsoleStream::Prefix(const std::string &_lbl,
                                    const std::string &_file,
                                    unsigned int _line,
                                    int _color)
{
  size_t index = _file.find_last_of("/") + 1;

  (void)_color;
  if (this->stream)
  {
#ifndef _WIN32
    *this->stream << "\033[1;" << _color << "m" << _lbl << " [" <<
      _file.substr(index , _file.size() - index) << ":" << _line <<
      "]\033[0m ";
#else
    *this->stream << _lbl << " [" <<
      _file.substr(index , _file.size() - index) << ":" << _line << "] ";
#endif
  }

  if (Console::Instance()->dataPtr->logFileStream.is_open())
  {
    Console::Instance()->dataPtr->logFileStream << _lbl << " [" <<
      _file.substr(index , _file.size() - index)<< ":" << _line << "] ";
  }
}
