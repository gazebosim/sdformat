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
#include <vector>

#include "sdf/system_util.hh"
#include "sdf/Error.hh"

#if defined(__GNUC__)
#define SDF_DEPRECATED(version) __attribute__((deprecated))
#define SDF_FORCEINLINE __attribute__((always_inline))
#elif defined(MSVC)
#define SDF_DEPRECATED(version)
#define SDF_FORCEINLINE __forceinline
#else
#define SDF_DEPRECATED(version)
#define SDF_FORCEINLINE
#endif

namespace sdf
{
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

  /// \brief Defines a color
  class SDFORMAT_VISIBLE Color
  {
    /// \brief Constructor
    /// \param[in] _r Red value (range 0 to 1)
    /// \param[in] _g Green value (range 0 to 1
    /// \param[in] _b Blue value (range 0 to 1
    /// \param[in] _a Alpha value (0=transparent, 1=opaque)
    /// \deprecated Use ignition::math::Color
    public: Color(float _r = 0.0f, float _g = 0.0f,
                  float _b = 0.0f, float _a = 1.0f) SDF_DEPRECATED(6.0)
            : r(_r), g(_g), b(_b), a(_a)
    {}

    /// \brief Stream insertion operator
    /// \param[in] _out the output stream
    /// \param[in] _pt the color
    /// \return the output stream
    public: friend std::ostream &operator<< (std::ostream &_out,
                                             const Color &_pt)
    {
      _out << _pt.r << " " << _pt.g << " " << _pt.b << " " << _pt.a;
      return _out;
    }

    /// \brief Stream insertion operator
    /// \param[in] _in the input stream
    /// \param[in] _pt
    public: friend std::istream &operator>> (std::istream &_in, Color &_pt)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _pt.r >> _pt.g >> _pt.b >> _pt.a;
      return _in;
    }

    /// \brief Equality operator
    /// \param[in] _clr The color to check for equality
    /// \return True if the this color equals _clf
    public: bool operator ==(const Color &_clr) const
    {
      return equal(this->r, _clr.r) &&
        equal(this->g, _clr.g) &&
        equal(this->b, _clr.b) &&
        equal(this->a, _clr.a);
    }

    /// \brief Red value
    public: float r;

    /// \brief Green value
    public: float g;

    /// \brief Blue value
    public: float b;

    /// \brief Alpha value
    public: float a;
  };

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

  /// \brief A class for inertial information about a link.
  class SDFORMAT_VISIBLE Inertia
  {
    public: double mass;
  };
}
#endif
