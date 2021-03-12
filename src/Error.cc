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

#include "sdf/Error.hh"

using namespace sdf;

class sdf::Error::Implementation
{
  /// \brief The error code value.
  public: ErrorCode code = ErrorCode::NONE;

#ifdef _WIN32
  // Disable warning C4251 which is triggered by
  // std::string
  #pragma warning(push)
  #pragma warning(disable: 4251)
#endif
  /// \brief Description of the error.
  public: std::string message = "";

  /// \brief File path where the error was raised.
  public: std::string filePath = "";
#ifdef _WIN32
  #pragma warning(pop)
#endif

  /// \brief Line number in the file path where the error was raised.
  public: int lineNumber = -1;
};

/////////////////////////////////////////////////
Error::Error()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message)
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message,
             const std::string &_filePath, int _lineNumber)
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
  this->dataPtr->filePath = _filePath;
  this->dataPtr->lineNumber = _lineNumber;
}

/////////////////////////////////////////////////
ErrorCode Error::Code() const
{
  return this->dataPtr->code;
}

/////////////////////////////////////////////////
std::string Error::Message() const
{
  return this->dataPtr->message;
}

/////////////////////////////////////////////////
std::string Error::FilePath() const
{
  return this->dataPtr->filePath;
}

/////////////////////////////////////////////////
int Error::LineNumber() const
{
  return this->dataPtr->lineNumber;
}

/////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
Error::operator bool() const
{
  return this->dataPtr->code != ErrorCode::NONE;
}

/////////////////////////////////////////////////
bool Error::operator==(const bool _value) const
{
  return ((this->dataPtr->code != ErrorCode::NONE) && _value) ||
         ((this->dataPtr->code == ErrorCode::NONE) && !_value);
}
