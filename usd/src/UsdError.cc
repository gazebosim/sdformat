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

#include "sdf/usd/UsdError.hh"

#include <iostream>

using namespace sdf::usd;

class sdf::usd::UsdError::Implementation
{
  /// \brief The error code value.
  public: UsdErrorCode code = UsdErrorCode::NONE;

  /// \brief Description of the error.
  public: std::string message = "";

  /// \brief File path where the error was raised.
  public: std::optional<std::string> filePath = std::nullopt;

  /// \brief Line number in the file path where the error was raised.
  public: std::optional<int> lineNumber = std::nullopt;

  /// \brief SDF error
  public: std::optional<sdf::Error> sdfError = std::nullopt;
};

/////////////////////////////////////////////////
UsdError::UsdError()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
UsdError::UsdError(const UsdErrorCode _code, const std::string &_message)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
}

/////////////////////////////////////////////////
UsdError::UsdError(const UsdErrorCode _code, const std::string &_message,
                   const std::string &_filePath)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
UsdError::UsdError(const UsdErrorCode _code, const std::string &_message,
                   const std::string &_filePath, int _lineNumber)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = _code;
  this->dataPtr->message = _message;
  this->dataPtr->filePath = _filePath;
  this->dataPtr->lineNumber = _lineNumber;
}

/////////////////////////////////////////////////
UsdError::UsdError(const sdf::Error &_sdfError)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->code = UsdErrorCode::SDF_ERROR;
  this->dataPtr->sdfError = _sdfError;
}

/////////////////////////////////////////////////
UsdErrorCode UsdError::Code() const
{
  return this->dataPtr->code;
}

/////////////////////////////////////////////////
const std::string &UsdError::Message() const
{
  return this->dataPtr->message;
}

/////////////////////////////////////////////////
const std::optional<std::string> &UsdError::FilePath() const
{
  return this->dataPtr->filePath;
}

/////////////////////////////////////////////////
void UsdError::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
std::optional<int> UsdError::LineNumber() const
{
  return this->dataPtr->lineNumber;
}

/////////////////////////////////////////////////
void UsdError::SetLineNumber(int _lineNumber)
{
  this->dataPtr->lineNumber = _lineNumber;
}

/////////////////////////////////////////////////
std::optional<sdf::Error> UsdError::SdfError() const
{
  return this->dataPtr->sdfError;
}

/////////////////////////////////////////////////
UsdError::operator bool() const
{
  return this->dataPtr->code != UsdErrorCode::NONE;
}

/////////////////////////////////////////////////
bool UsdError::operator==(const bool _value) const
{
  return ((this->dataPtr->code != UsdErrorCode::NONE) && _value) ||
         ((this->dataPtr->code == UsdErrorCode::NONE) && !_value);
}

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE
{
namespace usd
{

/////////////////////////////////////////////////
std::ostream &operator<<(std::ostream &_out, const sdf::usd::UsdError &_err)
{
  if (_err.Code() == UsdErrorCode::SDF_ERROR)
  {
    // make sure that an SdfError was wrapped into the UsdError if the error
    // code indicates an SDF error
    if (_err.SdfError())
    {
      _out << _err.SdfError().value();
    }
    else
    {
      _out << "Error code is of type SDF_ERROR, but no sdf::Error object "
           << "has been wrapped into this UsdError object.";
    }

    return _out;
  }

  std::string pathInfo = "";

  if (_err.FilePath().has_value())
    pathInfo += _err.FilePath().value();

  if (_err.LineNumber().has_value())
    pathInfo += ":L" + std::to_string(_err.LineNumber().value());

  if (!pathInfo.empty())
    pathInfo = "[" + pathInfo + "]: ";

  _out << "Error Code "
       << static_cast<std::underlying_type<sdf::usd::UsdErrorCode>::type>(
              _err.Code())
       << ": " << pathInfo << "Msg: " << _err.Message();
  return _out;
}
}  // namespace usd
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
