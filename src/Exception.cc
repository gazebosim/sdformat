/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <cstdint>
#include <string>

#include "sdf/Console.hh"
#include "sdf/Exception.hh"

using namespace sdf;

class sdf::Exception::Implementation
{
  /// \brief The error function
  public: std::string file;

  /// \brief Line the error occured on
  public: std::int64_t line;

  /// \brief The error string
  public: std::string str;
};

//////////////////////////////////////////////////
Exception::Exception()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Exception::Exception(const char *_file, std::int64_t _line, std::string _msg)
  : Exception()
{
  this->dataPtr->file = _file;
  this->dataPtr->line = _line;
  this->dataPtr->str = _msg;
  this->Print();
}

//////////////////////////////////////////////////
void Exception::Print() const
{
  sdf::Console::Instance()->ColorMsg("Exception",
      this->dataPtr->file,
      static_cast<unsigned int>(this->dataPtr->line), 31) << *this;
}

//////////////////////////////////////////////////
std::string Exception::GetErrorFile() const
{
  return this->dataPtr->file;
}

//////////////////////////////////////////////////
std::string Exception::GetErrorStr() const
{
  return this->dataPtr->str;
}

//////////////////////////////////////////////////
InternalError::InternalError()
{
}

//////////////////////////////////////////////////
InternalError::InternalError(const char *_file, std::int64_t _line,
                             const std::string _msg) :
  Exception(_file, _line, _msg)
{
}

//////////////////////////////////////////////////
InternalError::~InternalError()
{
}

//////////////////////////////////////////////////
AssertionInternalError::AssertionInternalError(
    const char * _file, std::int64_t _line,
    const std::string _expr,
    const std::string _function,
    const std::string _msg) :
  InternalError(_file, _line,
      "SDF ASSERTION                     \n" +
      _msg                               + "\n" +
      "In function       : " + _function + "\n" +
      "Assert expression : " + _expr     + "\n")
{
}

//////////////////////////////////////////////////
AssertionInternalError::~AssertionInternalError()
{
}
