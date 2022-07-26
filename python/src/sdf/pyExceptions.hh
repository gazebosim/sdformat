/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
 */

#ifndef SDFORMAT_PYTHON_EXCEPTIONS_HH_
#define SDFORMAT_PYTHON_EXCEPTIONS_HH_

#include <exception>
#include <sstream>
#include <stdexcept>

#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
// This class is a simple wrapper for sdf::Errors objects. It does not need a
// python binding. Instead, it holds the `sdf::Errors` objects and the string
// representation to be used by the pybind11 exception translator.
class PySDFErrorsException : public std::exception
{
  public: explicit PySDFErrorsException(sdf::Errors _errors)
      : errors(std::move(_errors))
  {
    std::stringstream ss;
    ss << this->errors;
    this->errorString = ss.str();
  }

  public: const char *what() const noexcept override
  {
    return this->errorString.c_str();
  }

  public: const sdf::Errors &Errors() const
  {
    return this->errors;
  }

  private: sdf::Errors errors;
  private: std::string errorString;
};
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif  // SDFORMAT_PYTHON_EXCEPTIONS_HH_
