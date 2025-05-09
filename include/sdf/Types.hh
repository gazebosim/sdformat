/*
 * Copyright 2011 Nate Koenig
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
#ifndef SDFORMAT_TYPES_HH_
#define SDFORMAT_TYPES_HH_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sdf/config.hh"
#include "sdf/system_util.hh"
#include "sdf/Error.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  constexpr std::string_view kScopeDelimiter{"::"};

  /// \brief The source path replacement if it was parsed from a string,
  /// instead of a file.
  constexpr char kSdfStringSource[] = "<data-string>";

  /// \brief The source path replacement if the urdf was parsed from a string,
  /// instead of a file.
  constexpr char kUrdfStringSource[] = "<urdf-string>";

  /// \brief Split a string using the delimiter in splitter.
  /// \param[in] str       The string to split.
  /// \param[in] splitter  The delimiter to use.
  /// \return A vector of strings containing the split tokens.
  SDFORMAT_VISIBLE
  std::vector<std::string> split(const std::string &_str,
                                 const std::string &_splitter);

  /// \brief Trim leading and trailing whitespace from a string.
  /// \param[in] _in The string to trim.
  /// \return A string containing the trimmed value.
  SDFORMAT_VISIBLE
  std::string trim(const char *_in);

  /// \brief Trim leading and trailing whitespace from a string.
  /// \param[in] _in The string to trim.
  /// \return A string containing the trimmed value.
  SDFORMAT_VISIBLE
  std::string trim(const std::string &_in);

  /// \brief check if two values are equal, within a tolerance
  /// \param[in] _a the first value
  /// \param[in] _b the second value
  /// \param[in] _epsilon the tolerance
  template<typename T>
  inline bool equal(const T &_a, const T &_b,
                    const T &_epsilon = 1e-6f)
  {
    return std::fabs(_a - _b) <= _epsilon;
  }

  /// \brief A vector of Error.
  using Errors = std::vector<Error>;

  /// \brief Output operator for a collection of errors.
  /// \param[in,out] _out The output stream.
  /// \param[in] _err The errors to output.
  /// \return Reference to the given output stream
  SDFORMAT_VISIBLE std::ostream &operator<<(
      std::ostream &_out, const sdf::Errors &_errs);

  /// \brief A Time class, can be used to hold wall- or sim-time.
  /// stored as sec and nano-sec.
  class SDFORMAT_VISIBLE Time
  {
    /// \brief Constructor
    public: Time()
            : sec(0), nsec(0)
    {
    }

    /// \brief Constructor
    /// \param[in] _sec Seconds
    /// \param[in] _nsec Nanoseconds
    public: Time(int32_t _sec, int32_t _nsec)
            : sec(_sec), nsec(_nsec)
    {
    }

    /// \brief Stream insertion operator
    /// \param[in] _out the output stream
    /// \param[in] _time time to write to the stream
    /// \return the output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Time &_time)
    {
      _out << _time.sec << " " << _time.nsec;
      return _out;
    }

    /// \brief Stream extraction operator
    /// \param[in] _in the input stream
    /// \param[in] _time time to read from to the stream
    /// \return the input stream
    public: friend std::istream &operator>>(std::istream &_in,
                                            Time &_time)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _time.sec >> _time.nsec;
      return _in;
    }

    /// \brief Equal to operator.
    /// \param[in] _time the time to compare to.
    /// \return true if values are the same, false otherwise.
    public: bool operator ==(const Time &_time) const
    {
      return this->sec == _time.sec && this->nsec == _time.nsec;
    }

    /// \brief Seconds.
    public: int32_t sec;

    /// \brief Nanoseconds.
    public: int32_t nsec;
  };

  /// \brief Transforms a string to its lowercase equivalent
  /// \param[in] _in String to convert to lowercase
  /// \return Lowercase equivalent of _in.
  std::string SDFORMAT_VISIBLE lowercase(const std::string &_in);

  /// \brief Split a name into a two strings based on the '::' delimiter
  /// \param[in] _absoluteName The fully qualified absolute name
  /// \return A pair with the absolute name minus the leaf node name, and the
  /// leaf name
  SDFORMAT_VISIBLE
  std::pair<std::string, std::string> SplitName(
      const std::string &_absoluteName);

  /// \brief Join two strings with the '::' delimiter.
  /// This checks for edge cases and is safe to use with any valid names
  /// \param[in] _scopeName the left-hand-side component
  /// \param[in] _localName the right-hand-side component
  /// \return A full string with the names joined by the '::' delimiter.
  SDFORMAT_VISIBLE
  std::string JoinName(
      const std::string &_scopeName, const std::string &_localName);
  }
}
#endif
