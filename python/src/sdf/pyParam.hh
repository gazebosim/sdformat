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

#ifndef SDFORMAT_PYTHON_PARAM_HH_
#define SDFORMAT_PYTHON_PARAM_HH_

#include <pybind11/pybind11.h>

#include <variant>

#include "sdf/Param.hh"
#include "sdf/config.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
// Define a pybind11 wrapper for an sdf::Param
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineParam(pybind11::object module);

/// \brief Compute the suffix for `get_` and `set_` functions that are defined
/// for a set of types defined in ParamPrivate::ParamVariant
template <typename T>
std::string computeSuffix()
{
  // TypeToString returns a value with a space, which would not be a valid
  // python function name, so we override that here.
  if constexpr (std::is_same_v<T, unsigned int>)
    return "unsigned_int";
  return ParamPrivate::TypeToString<T>();
}

/// \brief Implementation for forEachParamType
///
/// Base template for a single type. See specialization below
template<typename T>
struct ForEachParamTypeImpl
{
  /// \brief Calls the passed in function with a default constructed `T` object,
  /// which is used to determine the type within the callback lambda.
  /// \tparam Func Callback function type (usually a lambda)
  /// \param[in] _func Callback function
  template <typename Func>
  void operator()(Func _func) const
  {
    _func(T{});
  }
};


/// \brief specialization of ForEachParamTypeImpl for std::variant
///
/// This template will call the passed in function for each type in std::variant
template<typename ...Ts>
struct ForEachParamTypeImpl<std::variant<Ts...>>
{
  /// \brief Passes the passed in function to the single type specialization of
  /// ForEachParamTypeImpl.
  /// \tparam Func Callback function type (usually a lambda)
  /// \param[in] _func Callback function
  template <typename Func>
  void operator()(Func _func) const
  {
    (ForEachParamTypeImpl<Ts>{}(_func), ...);
  }
};
/// \brief Helper template for creating bindings for template functions in
/// `sdf::Param` and `sdf::Element`.
///
/// Since only the types in `ParamPrivate::ParamVariant` are supported by
/// libsdformat, we only create bindings for the types in that `std::variant`.
///
/// \sa ForEachParamTypeImpl
/// Usage:
/// To define a binding for Foo::Bar<T> on cls where cls is a pybind11::class_
/// ```
///   forEachParamType(
///       [&cls](auto &&arg)
///       {
///         using T = std::decay_t<decltype(arg)>;
///         const std::string funcName = "bar_" + computeSuffix<T>()
///         cls.def(funcName.c_str, &Foo::Bar<T>)
///       });
/// ```
static constexpr ForEachParamTypeImpl<ParamPrivate::ParamVariant>
    forEachParamType = {};

}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif  // SDFORMAT_PYTHON_SCENE_HH_
