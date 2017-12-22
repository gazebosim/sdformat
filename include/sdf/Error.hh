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
#ifndef SDF_ERROR_HH_
#define SDF_ERROR_HH_

#include <string>
#include "sdf/system_util.hh"

#ifdef _WIN32
  // Disable warning C4251 which is triggered by
  // std::string
  #pragma warning(push)
  #pragma warning(disable: 4251)
#endif

namespace sdf
{
  /// \enum ErrorCode
  /// \brief Set of error codes.
  enum class ErrorCode
  {
    // \brief No error
    NONE = 0,

    /// \brief Indicates that an SDF attribute is missing.
    NO_ATTRIBUTE,

    /// \brief Indicates that reading an SDF file failed.
    READ_FILE,
  };

  class SDFORMAT_VISIBLE Error
  {
    /// \brief default constructor
    public: Error() = default;

    /// \brief Constructor.
    /// \param[in] _code The error code.
    /// \param[in] _message A description of the error.
    /// \sa ErrorCode.
    public: Error(const ErrorCode _code, const std::string &_message);

    /// \brief Get the error code.
    /// \return An error code.
    /// \sa ErrorCode.
    public: ErrorCode Code() const;

    /// \brief Get the error message, which is a description of the error.
    /// \return Error message.
    public: std::string Message() const;

    /// \brief Safe bool conversion.
    /// \return True if this Error's Code() != NONE. In otherwords, this is
    /// true when there is an error.
    public: explicit operator bool() const;

    /// \brief Compare this Error to a boolean value.
    /// \return True if the boolean evaluation of this Error equals _value.
    /// If _value == false, then true is returned when this Error's Code() is
    /// equal to NONE and false is returned otherwise. If _value == true,
    /// then true is returned when this Error's Code() is not equal to NONE
    /// and false is returned otherwise.
    /// \sa explicit operator bool() const
    public: bool operator==(const bool _value) const;

    /// \brief The error code value.
    private: ErrorCode code = ErrorCode::NONE;

    /// \brief Description of the error.
    private: std::string message = "";
  };
}

#ifdef _WIN32
  #pragma warning(pop)
#endif

#endif
