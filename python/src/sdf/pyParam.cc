/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "pyParam.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "pybind11_helpers.hh"
#include "sdf/Element.hh"
#include "sdf/Param.hh"
#include "sdf/PrintConfig.hh"
#include "sdf/config.hh"

using namespace pybind11::literals;
namespace py = pybind11;
namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{



template <typename T>
struct ForEachParamTypeImpl;

template<typename ...Ts>
struct ForEachParamTypeImpl<std::variant<Ts...>>
{
  template <typename Func>
  void operator()(Func _func) const
  {
    (ForEachParamTypeImpl<Ts>{}(_func), ...);
  }
};

template<typename T>
struct ForEachParamTypeImpl
{
  template <typename Func>
  void operator()(Func _func) const
  {
    _func(T{});
  }
};

static constexpr ForEachParamTypeImpl<ParamPrivate::ParamVariant>
    forEachParamType = {};

void defineParam(py::object module)
{
  using PyClassParam = py::class_<Param, ParamPtr>;
  auto paramClass = PyClassParam(module, "Param");

  paramClass
      .def(py::init(
               [](const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_description)
               {
                 Errors errors;
                 auto output = Param(_key, _typeName, _default, _required,
                                          errors, _description);
                 ThrowIfErrors(errors);
                 return output;
               }),
           "key"_a, "type_name"_a, "default"_a, "required"_a,
           "description"_a = "")
      .def(py::init(
               [](const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_minValue, const std::string &_maxValue,
                  const std::string &_description)
               {
                 Errors errors;
                 auto output =
                     Param(_key, _typeName, _default, _required, _minValue,
                                _maxValue, errors, _description);
                 ThrowIfErrors(errors);
                 return output;
               }),
           "key"_a, "type_name"_a, "default"_a, "required"_a, "min_value"_a,
           "max_value"_a, "description"_a = "")
      .def(py::init<const Param &>())
      .def("get_as_string",
           ErrorWrappedCast<const PrintConfig &>(&Param::GetAsString,
                                                 py::const_),
           "Get the value as a string.", "config"_a = PrintConfig())
      .def("get_default_as_string",
           ErrorWrappedCast<const PrintConfig &>(&Param::GetDefaultAsString,
                                                 py::const_),
           "Get the default value as a string", "config"_a = PrintConfig())
      .def("get_min_value_as_string",
           ErrorWrappedCast<const PrintConfig &>(&Param::GetMinValueAsString,
                                                 py::const_),
           "Get the minimum allowed value as a string.",
           "config"_a = PrintConfig())
      .def("set_from_string",
           ErrorWrappedCast<const std::string &, bool>(&Param::SetFromString),
           "Set the parameter value from a string.")
      .def("set_from_string",
           ErrorWrappedCast<const std::string &>(&Param::SetFromString),
           "Set the parameter value from a string.")
      .def("get_parent_element", &Param::GetParentElement,
           "Get the parent Element of this Param.")
      .def("set_parent_element",
           ErrorWrappedCast<ElementPtr>(&Param::SetParentElement),
           "Set the parent Element of this Param.")
      .def("reset", &Param::Reset, "Reset the parameter to the default value.")
      .def("reparse", ErrorWrappedCast<>(&Param::Reparse),
           "Set the parameter value from a string.")
      .def("get_key", &Param::GetKey, "Get the key value.")
      // is_type is defined below
      .def("get_type_name", &Param::GetTypeName, "Get the type name value.")
      .def("get_required", &Param::GetRequired,
           "Return whether the parameter is required.")
      .def("get_set", &Param::GetSet,
           "Return true if the parameter has been set.")
      .def("ignores_parent_element_attribute",
           &Param::IgnoresParentElementAttribute,
           "Return true if the parameter ignores the parent element's "
           "attributes, or if the parameter has no parent element.")
      .def("clone", &Param::Clone, "Clone the parameter.")
      .def(
          "set_update_func",
          [](Param &_self, const py::object &_func)
          { _self.SetUpdateFunc(_func); },
          "Set the update function. The updateFunc will be used to set the "
          "parameter's value when Param::Update is called.")
      .def("update", ErrorWrappedCast<>(&Param::Update),
           "Set the parameter's value using the update_func.")
      // set is defined below
      // get_any is not supported since std::any is not supported in pybind11.
      // get is defined below
      .def("set_description", &Param::SetDescription,
           "Set the description of the parameter.")
      .def("get_description", &Param::GetDescription,
           "Get the description of the parameter.")
      .def("validate_value",
           ErrorWrappedCast<>(&Param::ValidateValue, py::const_),
           "Validate the value against minimum and maximum allowed values");

  // Definitions for `is_type`, `get`, `get_default`, and `set`
  forEachParamType(
      [&paramClass](auto &&arg)
      {
        using T = std::decay_t<decltype(arg)>;
        const std::string nameWithSuffix = "is_type_" + computeSuffix<T>();
        paramClass.def(nameWithSuffix.c_str(), &Param::IsType<T>,
                       "Return true if the param is a particular type");
      });

  forEachParamType(
      [&paramClass](auto &&arg)
      {
        using T = std::decay_t<decltype(arg)>;
        const std::string nameWithSuffix = "get_" + computeSuffix<T>();
        paramClass.def(
            nameWithSuffix.c_str(),
            [](const Param &_self)
            {
              sdf::Errors errors;
              T value;
              bool rc = _self.Get<T>(value, errors);
              ThrowIfErrors(errors);
              return py::make_tuple(value, rc);
            },
            "Get the value of the parameter.");
      });

  forEachParamType(
      [&paramClass](auto &&arg)
      {
        using T = std::decay_t<decltype(arg)>;
        const std::string nameWithSuffix = "get_default_" + computeSuffix<T>();
        paramClass.def(
            nameWithSuffix.c_str(),
            [](const Param &_self)
            {
              sdf::Errors errors;
              T value;
              bool rc = _self.GetDefault<T>(value, errors);
              ThrowIfErrors(errors);
              return py::make_tuple(value, rc);
            },
            "Get the default value of the parameter.");
      });

  forEachParamType(
      [&paramClass](auto &&arg)
      {
        using T = std::decay_t<decltype(arg)>;
        const std::string nameWithSuffix = "set_" + computeSuffix<T>();
        paramClass.def(nameWithSuffix.c_str(),
                       ErrorWrappedCast<const T &>(&Param::Set<T>),
                       "Set the value of the parameter.");
      });
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
