/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef SDF_EXCEPTION_HH_
#define SDF_EXCEPTION_HH_

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ignition/utils/ImplPtr.hh>
#include <sdf/sdf_config.h>
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \addtogroup sdf
  /// \{

  /// \brief This macro logs an error to the throw stream and throws
  /// an exception that contains the file name and line number.
  #define sdfthrow(msg) {std::ostringstream throwStream;\
    throwStream << msg << std::endl << std::flush;\
    throw sdf::Exception(__FILE__, __LINE__, throwStream.str()); }

  /// \class Exception Exception.hh common/common.hh
  /// \brief Class for generating exceptions
  class SDFORMAT_VISIBLE Exception
  {
    /// \brief Constructor
    public: Exception();

    /// \brief Default constructor
    /// \param[in] _file File name
    /// \param[in] _line Line number where the error occurred
    /// \param[in] _msg Error message
    public: Exception(const char *_file,
                      std::int64_t _line,
                      std::string _msg);

    /// \brief Copy constructor
    /// \param[in] _e Exception to copy.
    public: Exception(const Exception &_e) = default;

    /// \brief Move constructor
    /// \param[in] _e Exception to move.
    public: Exception(Exception &&_e) noexcept = default;

    /// \brief Assignment operator.
    /// \param[in] _exception The exception to set values from.
    /// \return *this
    public: Exception &operator=(const Exception &_exception) = default;

    /// \brief Move assignment operator.
    /// \param[in] _exception Exception to move.
    /// \return Reference to this.
    public: Exception &operator=(Exception &&_exception) noexcept = default;

    /// \brief Destructor
    public: virtual ~Exception() = default;

    /// \brief Return the error function
    /// \return The error function name
    public: std::string GetErrorFile() const;

    /// \brief Return the error string
    /// \return The error string
    public: std::string GetErrorStr() const;

    /// \brief Print the exception to std out.
    public: void Print() const;


    /// \brief stream insertion operator for Gazebo Error
    /// \param[in] _out the output stream
    /// \param[in] _err the exception
    public: friend std::ostream &operator<<(std::ostream& _out,
                                            const sdf::Exception &_err)
    {
      return _out << _err.GetErrorStr();
    }

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };

  /// \class InternalError Exception.hh common/common.hh
  /// \brief Class for generating Internal Gazebo Errors:
  ///        those errors which should never happend and
  ///        represent programming bugs.
  class SDFORMAT_VISIBLE InternalError : public Exception
  {
    /// \brief Constructor
    public: InternalError();

    /// \brief Default constructor
    /// \param[in] _file File name
    /// \param[in] _line Line number where the error occurred
    /// \param[in] _msg Error message
    public: InternalError(const char *_file, std::int64_t _line,
                          const std::string _msg);

    /// \brief Destructor
    public: virtual ~InternalError();
  };

  /// \class AssertionInternalError Exception.hh common/common.hh
  /// \brief Class for generating Exceptions which come from
  ///        sdf assertions. They include information about the
  ///        assertion expression violated, function where problem
  ///        appeared and assertion debug message.
  class SDFORMAT_VISIBLE AssertionInternalError : public InternalError
  {
    /// \brief Constructor for assertions
    /// \param[in] _file File name
    /// \param[in] _line Line number where the error occurred
    /// \param[in] _expr Assertion expression failed resulting in an
    ///                  internal error
    /// \param[in] _function Function where assertion failed
    /// \param[in] _msg Function where assertion failed
    public: AssertionInternalError(const char *_file,
                                   std::int64_t _line,
                                   const std::string _expr,
                                   const std::string _function,
                                   const std::string _msg = "");
    /// \brief Destructor
    public: virtual ~AssertionInternalError();
  };
  /// \}
  }
}

#endif
