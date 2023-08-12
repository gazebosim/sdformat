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
#ifndef SDFORMAT_PYTHON_PYBIND11_HELPERS_HH_
#define SDFORMAT_PYTHON_PYBIND11_HELPERS_HH_

#include <sdf/sdf_config.h>

#include <sdf/Types.hh>

#include "pyExceptions.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE
{
namespace python
{
/// \brief Throw an exception if the `_errors` container is not empty.
/// \param[in] _errors The list of errors to check.
/// \throws PySDFErrorsException
void ThrowIfErrors(const sdf::Errors &_errors);


// NOTE: This currently only works for member funtions
template <typename... Args>
struct ErrorWrappedCastImpl
{
  template <typename Return, typename Class, typename Func>
  static auto ErrorFirst(Func pmf)
  {
    return [pmf](Class &_self, Args... args)
    {
      sdf::Errors errors;
      if constexpr (std::is_same_v<Return, void>)
      {
        (_self.*pmf)(errors, std::forward<Args>(args)...);
        ThrowIfErrors(errors);
      }
      else
      {
        auto output = (_self.*pmf)(errors, std::forward<Args>(args)...);
        ThrowIfErrors(errors);
        return output;
      }
    };
  }

  template <typename Return, typename Class, typename Func>
  static auto ErrorLast(Func pmf)
  {
    return [pmf](Class &_self, Args... args)
    {
      sdf::Errors errors;
      if constexpr (std::is_same_v<Return, void>)
      {
        (_self.*pmf)(std::forward<Args>(args)..., errors);
        ThrowIfErrors(errors);
      }
      else
      {
        auto output = (_self.*pmf)(std::forward<Args>(args)..., errors);
        ThrowIfErrors(errors);
        return output;
      }
    };
  }

  template <typename Return, typename Class>
  auto operator()(Return (Class::*pmf)(Errors &, Args...) const,
                  std::true_type) const noexcept
  {
    return ErrorFirst<Return, Class>(pmf);
  }
  template <typename Return, typename Class>
  auto operator()(Return (Class::*pmf)(Errors &, Args...),
                  std::false_type = {}) const noexcept
  {
    return ErrorFirst<Return, Class>(pmf);
  }

  template <typename Return, typename Class,
            std::size_t nArgs = sizeof...(Args),
            typename = std::enable_if_t<nArgs != 0>>
  auto operator()(Return (Class::*pmf)(Args..., Errors &) const,
                  std::true_type) const noexcept
  {
    return ErrorLast<Return, Class>(pmf);
  }

  template <typename Return, typename Class,
            std::size_t nArgs = sizeof...(Args),
            typename = std::enable_if_t<nArgs != 0>>
  auto operator()(Return (Class::*pmf)(Args..., Errors &),
                  std::false_type = {}) const noexcept
  {
    return ErrorLast<Return, Class>(pmf);
  }
};

template <typename ...Args>
static constexpr ErrorWrappedCastImpl<Args...> ErrorWrappedCast = {};

}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif 
