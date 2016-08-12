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

#ifndef _SDF_TYPES_HH_
#define _SDF_TYPES_HH_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>

#include "sdf/system_util.hh"

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
  /// \brief Windows equivalent of getEnv.
  /// Should only be called when using windows.
  /// \param[in] _name Name of the environment variable to get.
  /// \return Environment variable contents, or NULL on error.
  SDFORMAT_VISIBLE
  const char *winGetEnv(const char *_name);

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

  /// \brief Defines a color
  class SDFORMAT_VISIBLE Color
  {
    /// \brief Constructor
    /// \param[in] _r Red value (range 0 to 1)
    /// \param[in] _g Green value (range 0 to 1
    /// \param[in] _b Blue value (range 0 to 1
    /// \param[in] _a Alpha value (0=transparent, 1=opaque)
    public: Color(float _r = 0.0f, float _g = 0.0f,
                  float _b = 0.0f, float _a = 1.0f)
            : r(_r), g(_g), b(_b), a(_a) {}

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

  /// \deprecated Use ignition::math::Vector2i
  /// \brief Generic integer x, y vector
  class SDFORMAT_VISIBLE Vector2i
  {
    /// \brief Constructor
    /// \param[in] _x value along x
    /// \param[in] _y value along y
    public: Vector2i(int _x = 0, int _y = 0)
            : x(_x), y(_y) {}

    /// \brief Stream insertion operator
    /// \param[in] _out output stream
    /// \param[in] pt Vector2i to output
    /// \return the stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                const Vector2i &_pt)
    {
      _out << _pt.x << " " << _pt.y;
      return _out;
    }

    /// \brief Stream extraction operator
    /// \param[in] _in input stream
    /// \param[in] _pt Vector2i to read values into
    /// \return The stream
    public: friend std::istream &operator>>(std::istream &_in,
                Vector2i &_pt)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _pt.x >> _pt.y;
      return _in;
    }

    /// \brief Equality operator
    /// \param _pt The vector to compare with
    /// \return True if component have the same values, false otherwise
    public: bool operator ==(const Vector2i &_pt) const
            {
              return this->x == _pt.x && this->y == _pt.y;
            }

    /// \brief x data
    public: int x;

    /// \brief y data
    public: int y;
  } SDF_DEPRECATED(4.0);

  /// \deprecated Use ignition::math::Vector2d
  /// \brief Generic double x, y vector
  class SDFORMAT_VISIBLE Vector2d
  {
    /// \brief Constructor
    /// \param[in] _x value along x
    /// \param[in] _y value along y
    public: Vector2d(double _x = 0.0, double _y = 0.0)
            : x(_x), y(_y) {}

    /// \brief Stream extraction operator
    /// \param[in] _out output stream
    /// \param[in] _pt Vector2d to output
    /// \return The stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                const Vector2d &_pt)
    {
      _out << _pt.x << " " << _pt.y;
      return _out;
    }

    /// \brief Stream extraction operator
    /// \param[in] _in input stream
    /// \param[in] _pt Vector2d to read values into
    /// \return The stream
    public: friend std::istream &operator>>(std::istream &_in,
                Vector2d &_pt)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _pt.x >> _pt.y;
      return _in;
    }

    /// \brief Equal to operator
    /// \param[in] _v the vector to compare to
    /// \return true if the elements of the 2 vectors are equal within
    /// a tolerence (1e-6)
    public: bool operator ==(const Vector2d &_pt) const
            {
              return equal(this->x, _pt.x) && equal(this->y, _pt.y);
            }


    /// \brief x data
    public: double x;

    /// \brief y data
    public: double y;
  } SDF_DEPRECATED(4.0);

  /// \deprecated Use ignition::math::Vector3d
  /// \brief The Vector3 class represents the generic vector containing 3
  ///        elements.  Since it's commonly used to keep coordinate system
  ///        related information, its elements are labeled by x, y, z.
  class SDFORMAT_VISIBLE Vector3
  {
    /// \brief Copy constructor
    /// \param[in] _v a vector
    public: Vector3(const Vector3 &_v)
            : x(_v.x), y(_v.y), z(_v.z) {}

    /// \brief Constructor
    /// \param[in] _x value along x
    /// \param[in] _y value along y
    /// \param[in] _z value along z
    public: Vector3(double _x = 0.0, double _y = 0.0, double _z = 0.0)
            : x(_x), y(_y), z(_z) {}

    /// \brief Assignment operator
    /// \param[in] _v a new value
    /// \return this
    public: Vector3 &operator=(const Vector3 &_v)
    {
      if (this == &_v)
        return *this;

      this->x = _v.x;
      this->y = _v.y;
      this->z = _v.z;

      return *this;
    }

    /// \brief Addition operator
    /// \param[in] _v vector to add
    /// \return the sum vector
    public: Vector3 operator+(const Vector3 &_v) const
    {
      return Vector3(this->x + _v.x, this->y + _v.y, this->z + _v.z);
    }

    /// \brief Return the cross product of this vector and pt
    /// \return the product
    public: Vector3 Cross(const Vector3 &_pt) const
            {
              Vector3 c(0, 0, 0);

              c.x = this->y * _pt.z - this->z * _pt.y;
              c.y = this->z * _pt.x - this->x * _pt.z;
              c.z = this->x * _pt.y - this->y * _pt.x;

              return c;
            }

    /// \brief Multiplication operator
    /// \remarks this is an element wise multiplication, not a cross product
    /// \param[in] _v
    public: Vector3 operator*(double _v) const
            {
              return Vector3(this->x * _v, this->y * _v, this->z * _v);
            }

    /// \brief Stream insertion operator
    /// \param _out output stream
    /// \param _pt Vector3 to output
    /// \return the stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Vector3 &_pt)
    {
      _out << _pt.x << " " << _pt.y << " " << _pt.z;
      return _out;
    }

    /// \brief Multiplication by a double
    /// \param[in] _v A double
    /// \return this
    public: const Vector3 &operator*=(double _v)
            {
              this->x *= _v;
              this->y *= _v;
              this->z *= _v;

              return *this;
            }

    /// \brief Equal to operator
    /// \param[in] _pt The vector to compare against
    /// \return true if each component is equal within a
    /// tolerence (1e-3), false otherwise
    public: bool operator==(const sdf::Vector3 &_pt) const
            {
              return equal(this->x, _pt.x, 0.001) &&
                     equal(this->y, _pt.y, 0.001) &&
                     equal(this->z, _pt.z, 0.001);
            }

    /// \brief Stream extraction operator
    /// \param _in input stream
    /// \param _pt vector3 to read values into
    /// \return the stream
    public: friend std::istream &operator>>(std::istream &_in,
                                            Vector3 &_pt)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _pt.x >> _pt.y >> _pt.z;
      return _in;
    }

    /// \brief x Data
    public: double x;

    /// \brief y Data
    public: double y;

    /// \brief z Data
    public: double z;
  } SDF_DEPRECATED(4.0);

  /// \deprecated Use ignition::math::Quaterniond
  /// \brief A quaternion class
  class SDFORMAT_VISIBLE Quaternion
  {
    /// \brief Default Constructor
    public: Quaternion() : x(0), y(0), z(0), w(1)
            {}

    /// \brief Copy constructor
    /// \param[in] _q Quaternion to copy
    public: Quaternion(const Quaternion &_q)
            : x(_q.x), y(_q.y), z(_q.z), w(_q.w) {}

    public: Quaternion(const double &_roll, const double &_pitch,
                const double &_yaw)
            {
              this->SetFromEuler(Vector3(_roll, _pitch, _yaw));
            }

    /// \brief Constructor
    /// \param[in] _w W param
    /// \param[in] _x X param
    /// \param[in] _y Y param
    /// \param[in] _z Z param
    public: Quaternion(double _w, double _x, double _y, double _z)
            : x(_x), y(_y), z(_z), w(_w) {}

    /// \brief Convert euler angles to quatern.
    /// \param[in] _x rotation along x
    /// \param[in] _y rotation along y
    /// \param[in] _z rotation along z
    public: static Quaternion EulerToQuaternion(double _x, double _y, double _z)
            {
              return EulerToQuaternion(Vector3(_x, _y, _z));
            }

    /// \brief Convert euler angles to quatern.
    /// \param[in] _vec Vector of Euler angles
    public: static Quaternion EulerToQuaternion(const Vector3 &_vec)
            {
              Quaternion result;
              result.SetFromEuler(_vec);
              return result;
            }

    /// \brief Equal operator
    /// \param[in] _qt Quaternion to copy
    public: Quaternion &operator =(const Quaternion &_qt)
            {
              this->w = _qt.w;
              this->x = _qt.x;
              this->y = _qt.y;
              this->z = _qt.z;

              return *this;
            }

    /// \brief Multiplication operator
    /// \param[in] _qt Quaternion for multiplication
    /// \return This quaternion multiplied by the parameter
    public: inline Quaternion operator*(const Quaternion &_q) const
            {
              return Quaternion(
                  this->w*_q.w - this->x*_q.x - this->y*_q.y - this->z*_q.z,
                  this->w*_q.x + this->x*_q.w + this->y*_q.z - this->z*_q.y,
                  this->w*_q.y - this->x*_q.z + this->y*_q.w + this->z*_q.x,
                  this->w*_q.z + this->x*_q.y - this->y*_q.x + this->z*_q.w);
            }

    /// \brief Get the inverse of this quaternion
    /// \return Inverse quarenion
    public: inline Quaternion GetInverse() const
            {
              double s = 0;
              Quaternion q(this->w, this->x, this->y, this->z);

              // use s to test if quaternion is valid
              s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

              if (equal(s, 0.0))
              {
                q.w = 1.0;
                q.x = 0.0;
                q.y = 0.0;
                q.z = 0.0;
              }
              else
              {
                // deal with non-normalized quaternion
                // div by s so q * qinv = identity
                q.w =  q.w / s;
                q.x = -q.x / s;
                q.y = -q.y / s;
                q.z = -q.z / s;
              }
              return q;
            }

    /// \brief Return the rotation in Euler angles
    /// \return This quaternion as an Euler vector
    public: Vector3 GetAsEuler() const
            {
              Vector3 vec;

              Quaternion copy = *this;
              double squ;
              double sqx;
              double sqy;
              double sqz;

              copy.Normalize();

              squ = copy.w * copy.w;
              sqx = copy.x * copy.x;
              sqy = copy.y * copy.y;
              sqz = copy.z * copy.z;

              // Roll
              vec.x = atan2(2 * (copy.y*copy.z + copy.w*copy.x),
                  squ - sqx - sqy + sqz);

              // Pitch
              double sarg = -2 * (copy.x*copy.z - copy.w * copy.y);
              vec.y = sarg <= -1.0 ? -0.5*M_PI :
                (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));

              // Yaw
              vec.z = atan2(2 * (copy.x*copy.y + copy.w*copy.z),
                  squ + sqx - sqy - sqz);

              return vec;
            }

    /// \brief Set the quaternion from Euler angles
    /// \param[in] _vec  Euler angle
    public: void SetFromEuler(const Vector3 &_vec)
            {
              double phi, the, psi;

              phi = _vec.x / 2.0;
              the = _vec.y / 2.0;
              psi = _vec.z / 2.0;

              this->w = cos(phi) * cos(the) * cos(psi) + sin(phi) *
                sin(the) * sin(psi);
              this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) *
                sin(the) * sin(psi);
              this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) *
                cos(the) * sin(psi);
              this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) *
                sin(the) * cos(psi);

              this->Normalize();
            }

    /// \brief Normalize the quaternion
    public: void Normalize()
            {
              double s = 0;

              s = sqrt(this->w * this->w + this->x * this->x +
                  this->y * this->y +
                  this->z * this->z);

              if (equal(s, 0.0))
              {
                this->w = 1.0;
                this->x = 0.0;
                this->y = 0.0;
                this->z = 0.0;
              }
              else
              {
                this->w /= s;
                this->x /= s;
                this->y /= s;
                this->z /= s;
              }
            }

    /// \brief Rotate a vector using the quaternion
    /// \param[in] _vec vector to rotate
    /// \return the rotated vector
    public: inline Vector3 RotateVector(const Vector3 &_vec) const
            {
              Quaternion tmp(0.0, _vec.x, _vec.y, _vec.z);
              tmp = (*this) * (tmp * this->GetInverse());
              return Vector3(tmp.x, tmp.y, tmp.z);
            }

    /// \brief Stream insertion operator
    /// \param[in] _out output stream
    /// \param[in] _q quaternion to output
    /// \return the stream
    public: friend  std::ostream &operator<<(std::ostream &_out,
                const Quaternion &_q)
    {
      Vector3 v(_q.GetAsEuler());
      _out << v.x << " " << v.y << " " << v.z;
      return _out;
    }

    /// \brief Vector3 multiplication operator
    /// \param[in] _v vector to multiply
    public: Vector3 operator*(const Vector3 &_v) const
            {
              Vector3 uv, uuv;
              Vector3 qvec(this->x, this->y, this->z);
              uv = qvec.Cross(_v);
              uuv = qvec.Cross(uv);
              uv *= (2.0f * this->w);
              uuv *= 2.0f;

              return _v + uv + uuv;
            }

    /// \brief Stream extraction operator
    /// \param[in] _in input stream
    /// \param[in] _q Quaternion to read values into
    /// \return The istream
    public: friend std::istream &operator>>(std::istream &_in,
                                             Quaternion &_q)
    {
      double roll, pitch, yaw;

      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> roll >> pitch >> yaw;

      _q.SetFromEuler(Vector3(roll, pitch, yaw));

      return _in;
    }

    /// \brief Equal to operator
    /// \param[in] _qt Quaternion for comparison
    /// \return True if equal
    public: bool operator ==(const Quaternion &_qt) const
            {
              return this->GetAsEuler() == _qt.GetAsEuler();
            }

    public: inline void Correct()
            {
              if (!std::isfinite(this->x))
                this->x = 0;
              if (!std::isfinite(this->y))
                this->y = 0;
              if (!std::isfinite(this->z))
                this->z = 0;
              if (!std::isfinite(this->w))
                this->w = 1;

              if (equal(this->w, 0.0) && equal(this->x, 0.0) &&
                  equal(this->y, 0.0) && equal(this->z, 0.0))
              {
                this->w = 1;
              }
            }

    /// \brief x data
    public: double x;

    /// \brief y data
    public: double y;

    /// \brief z data
    public: double z;

    /// \brief w data
    public: double w;
  } SDF_DEPRECATED(4.0);

  /// \deprecated Use ignition::math::Pose3d
  /// \brief Encapsulates a position and rotation in three space
  class SDFORMAT_VISIBLE Pose
  {
    /// \brief Constructor
    public: Pose()
            :pos(0, 0, 0), rot(1, 0, 0, 0)
            {}

    /// \brief Constructor
    /// \param[in] _pos A position
    /// \param[in] _rot A rotation
    public: Pose(Vector3 _pos, Quaternion _rot)
            : pos(_pos), rot(_rot) {}

    /// \brief Constructor
    /// \param[in] _x x position in meters.
    /// \param[in] _y y position in meters.
    /// \param[in] _z z position in meters.
    /// \param[in] _roll Roll (rotation about X-axis) in radians.
    /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
    /// \param[in] _yaw Yaw (rotation about z-axis) in radians.
    public: Pose(double _x, double _y, double _z,
                 double _roll, double _pitch, double _yaw)
            : pos(_x, _y, _z), rot(_roll, _pitch, _yaw)
            {
              rot.Correct();
            }

    /// \brief Stream insertion operator
    /// \param out output stream
    /// \param pose pose to output
    /// \return the stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Pose &_pose)
            {
              _out << _pose.pos << " " << _pose.rot;
              return _out;
            }

    /// \brief Stream extraction operator
    /// \param[in] _in the input stream
    /// \param[in] _pose the pose
    /// \return the stream
    public: friend std::istream &operator>>(std::istream &_in,
              Pose &_pose)
          {
            // Skip white spaces
            _in.setf(std::ios_base::skipws);
            _in >> _pose.pos >> _pose.rot;
            return _in;
          }

    /// \brief Multiplication operator
    /// \param[in] _pose the other pose
    /// \return itself
    public: Pose operator*(const Pose &pose)
            {
              return Pose(this->CoordPositionAdd(pose),  pose.rot * this->rot);
            }

    /// \brief Add one point to another: result = this + pose
    /// \param[in] _pose The Pose to add
    /// \return The resulting position
    public: Vector3 CoordPositionAdd(const Pose &_pose) const
            {
              Quaternion tmp;
              Vector3 result;

              // result = _pose.rot + _pose.rot * this->pos * _pose.rot!
              tmp.w = 0.0;
              tmp.x = this->pos.x;
              tmp.y = this->pos.y;
              tmp.z = this->pos.z;

              tmp = _pose.rot * (tmp * _pose.rot.GetInverse());

              result.x = _pose.pos.x + tmp.x;
              result.y = _pose.pos.y + tmp.y;
              result.z = _pose.pos.z + tmp.z;

              return result;
            }

    /// \brief Equality operator
    /// \param[in] _pose Pose for comparison
    /// \return True if equal
    public: bool operator ==(const Pose &_pose) const
            {
              return this->pos == _pose.pos && this->rot == _pose.rot;
            }

    /// \brief Position data
    public: Vector3 pos;

    /// \brief Orientation data
    public: Quaternion rot;
  } SDF_DEPRECATED(4.0);

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
