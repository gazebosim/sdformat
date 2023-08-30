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

#include "sdf/Assert.hh"
#include "sdf/Error.hh"

using namespace sdf;

class sdf::Error::Implementation
{
  /// \brief The error code value.
  public: ErrorCode code = ErrorCode::NONE;

  /// \brief Description of the error.
  public: std::string message = "";

  /// \brief Xml path where the error was raised.
  public: std::optional<std::string> xmlPath = std::nullopt;

  /// \brief File path where the error was raised.
  public: std::optional<std::string> filePath = std::nullopt;

  /// \brief Line number in the file path where the error was raised.
  public: std::optional<int> lineNumber = std::nullopt;
};

/////////////////////////////////////////////////
Error::Error()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message)
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message,
             const std::string &_filePath)
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message,
             const std::string &_filePath, int _lineNumber)
  : dataPtr(gz::utils::MakeImpl<Implementation>())
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
void Error::SetMessage(const std::string &_message)
{
  this->dataPtr->message = _message;
}

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
std::optional<std::string> Error::XmlPath() const
{
  return this->dataPtr->xmlPath;
}

/////////////////////////////////////////////////
void Error::SetXmlPath(const std::string &_xmlPath)
{
  this->dataPtr->xmlPath = _xmlPath;
}

/////////////////////////////////////////////////
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

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
std::ostream &operator<<(std::ostream &_out, const sdf::Error &_err)
{
  std::string pathInfo = "";

  if (_err.XmlPath().has_value())
    pathInfo += _err.XmlPath().value();

  if (_err.FilePath().has_value())
    pathInfo += ":" + _err.FilePath().value();

  if (_err.LineNumber().has_value())
    pathInfo += ":L" + std::to_string(_err.LineNumber().value());

  if (!pathInfo.empty())
    pathInfo = "[" + pathInfo + "]: ";

  _out << "Error Code "
      << static_cast<std::underlying_type<sdf::ErrorCode>::type>(
          _err.Code()) << ": "
      << pathInfo
      << "Msg: " << _err.Message();
  return _out;
}

namespace internal
{

void throwOrPrintError(sdf::Console::ConsoleStream &_out,
                       const sdf::Error &_error)
{
  if (_error.Code() == sdf::ErrorCode::FATAL_ERROR)
  {
    SDF_ASSERT(false, _error.Message());
  }
  else
  {
    _out << _error.Message();
  }
}
}  // namespace internal
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
