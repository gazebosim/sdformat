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

#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

#include "sdf/Console.hh"

using namespace sdf;

boost::shared_ptr<Console> Console::myself;
static boost::mutex g_instance_mutex;

//////////////////////////////////////////////////
Console::Console()
  : msgStream(&std::cerr), logStream(NULL)
{
  // Set up the file that we'll log to.
  try
  {
    char* home = getenv("HOME");
    if (!home)
    {
      sdfwarn << "No HOME defined in the environment. Will not log.";
      return;
    }
    boost::filesystem::path logFile(home);
    logFile /= ".sdformat";
    logFile /= "sdformat.log";
    boost::filesystem::path logDir = logFile.parent_path();
    if (!boost::filesystem::exists(logDir))
    {
      boost::filesystem::create_directory(logDir);
    }
    else if (!boost::filesystem::is_directory(logDir))
    {
      sdfwarn << logDir << " exists but is not a directory.  Will not log.";
      return;
    }
    this->logFileStream.open(logFile.string().c_str(), std::ios::out);
  }
  catch(const boost::filesystem::filesystem_error& e)
  {
    sdfwarn << "Exception while setting up logging: " << e.what();
    return;
  }
}

//////////////////////////////////////////////////
Console::~Console()
{
}

//////////////////////////////////////////////////
boost::shared_ptr<Console> Console::Instance()
{
  boost::mutex::scoped_lock lock(g_instance_mutex);
  if (!myself)
    myself.reset(new Console());

  return myself;
}

//////////////////////////////////////////////////
void Console::SetQuiet(bool)
{
}

//////////////////////////////////////////////////
Console::ConsoleStream &Console::ColorMsg(const std::string &lbl,
                                          const std::string &file,
                                          unsigned int line, int color)
{
  this->msgStream.Prefix(lbl, file, line, color);
  return this->msgStream;
}

//////////////////////////////////////////////////
Console::ConsoleStream &Console::Log(const std::string &lbl,
                                     const std::string &file,
                                     unsigned int line)
{
  this->logStream.Prefix(lbl, file, line, 0);
  return this->logStream;
}
