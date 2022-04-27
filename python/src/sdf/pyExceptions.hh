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
#include <stdexcept>

#include <pybind11/pybind11.h>

#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
class SDFErrorsException : public std::exception
{
public:
  explicit SDFErrorsException(const sdf::Errors &_errors)
  : errors(_errors)
  {
  }

  std::string getErrorString()
  {
    std::string errorStr = "\n";
    for (const auto & error : this->errors)
    {
      errorStr += std::string("Error code: ") +
                  std::to_string(static_cast<int>(error.Code()));
      auto lineNumber = error.LineNumber();
      if (lineNumber)
      {
        errorStr += std::string("Line: ") +
                    std::to_string(error.LineNumber().value());
      }
      errorStr += std::string(" Message: ") + error.Message() + "\n";
    }
    return errorStr;
  }

  sdf::Errors GetErrors()
  {
    return this->errors;
  }

  ~SDFErrorsException() = default;

private:
  sdf::Errors errors;
};

static PyObject *SDFErrorsException_tp_str(PyObject *selfPtr)
{
	pybind11::str ret;
	try {
		pybind11::handle self(selfPtr);
		pybind11::tuple args = self.attr("args");
		ret = pybind11::str(args[0]);
	} catch (pybind11::error_already_set &e) {
		ret = "";
	}

	/* ret will go out of scope when returning, therefore increase its reference
	count, and transfer it to the caller (like PyObject_Str). */
	ret.inc_ref();
	return ret.ptr();
}

static PyObject *SDFErrorsException_geterrors(PyObject *selfPtr, void */*closure*/)
{
	try {
		pybind11::handle self(selfPtr);
		pybind11::tuple args = self.attr("args");
		pybind11::object code = args[1];
		code.inc_ref();
		return  code.ptr();
	} catch (pybind11::error_already_set &e) {
		/* We could simply backpropagate the exception with e.restore, but
		exceptions like OSError return None when an attribute is not set. */
		pybind11::none ret;
		ret.inc_ref();
		return ret.ptr();
	}
}

static PyGetSetDef SDFErrorsException_getsetters[] = {
	{"errors", SDFErrorsException_geterrors, NULL, NULL, NULL},
	{NULL}
};
/// Define a pybind11 wrapper for Exceptions
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineExceptions(pybind11::object module);
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif  // SDFORMAT_PYTHON_EXCEPTIONS_HH_
