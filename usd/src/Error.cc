/*
 * Copyright 2022 Open Source Robotics Foundation
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

#include "sdf/usd/Error.hh"

#include <iostream>

using namespace sdf::usd;

class sdf::usd::Error::Implementation
{
  /// \brief The error code value.
 public:
  ErrorCode code = ErrorCode::NONE;

  /// \brief Description of the error.
 public:
  std::string message = "";

  /// \brief File path where the error was raised.
 public:
  std::optional<std::string> filePath = std::nullopt;

  /// \brief Line number in the file path where the error was raised.
 public:
  std::optional<int> lineNumber = std::nullopt;

 public:
  std::optional<sdf::Error> sdf_error = std::nullopt;
};

/////////////////////////////////////////////////
Error::Error() : dataPtr(ignition::utils::MakeImpl<Implementation>()) {}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message,
             const std::string &_filePath)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
  this->dataPtr->filePath = _filePath;
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
Error::Error(const sdf::Error &_sdf_error)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = ErrorCode::SDF_ERROR;
  this->dataPtr->sdf_error = _sdf_error;
}

/////////////////////////////////////////////////
ErrorCode Error::Code() const { return this->dataPtr->code; }

/////////////////////////////////////////////////
std::string Error::Message() const { return this->dataPtr->message; }

/////////////////////////////////////////////////
std::optional<std::string> Error::FilePath() const
{
  return this->dataPtr->filePath;
}

/////////////////////////////////////////////////
void Error::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
std::optional<int> Error::LineNumber() const
{
  return this->dataPtr->lineNumber;
}

/////////////////////////////////////////////////
void Error::SetLineNumber(int _lineNumber)
{
  this->dataPtr->lineNumber = _lineNumber;
}

/////////////////////////////////////////////////
std::optional<sdf::Error> Error::SdfError() const
{
  return this->dataPtr->sdf_error;
}

/////////////////////////////////////////////////
Error::operator bool() const { return this->dataPtr->code != ErrorCode::NONE; }

/////////////////////////////////////////////////
bool Error::operator==(const bool _value) const
{
  return ((this->dataPtr->code != ErrorCode::NONE) && _value) ||
         ((this->dataPtr->code == ErrorCode::NONE) && !_value);
}

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE
{
namespace usd
{

/////////////////////////////////////////////////
std::ostream &operator<<(std::ostream &_out, const sdf::usd::Error &_err)
{
  if (_err.dataPtr->code == ErrorCode::SDF_ERROR)
  {
    _out << _err.dataPtr->sdf_error.value();
    return _out;
  }

  std::string pathInfo = "";

  if (_err.FilePath().has_value()) pathInfo += ":" + _err.FilePath().value();

  if (_err.LineNumber().has_value())
    pathInfo += ":L" + std::to_string(_err.LineNumber().value());

  if (!pathInfo.empty()) pathInfo = "[" + pathInfo + "]: ";

  _out << "Error Code "
       << static_cast<std::underlying_type<sdf::usd::ErrorCode>::type>(
              _err.Code())
       << ": " << pathInfo << "Msg: " << _err.Message();
  return _out;
}
}  // namespace usd
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
