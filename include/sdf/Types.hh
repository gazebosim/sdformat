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

#if defined(__GNUC__)
#define SDF_DEPRECATED __attribute__((deprecated))
#define SDF_FORCEINLINE __attribute__((always_inline))
#elif defined(MSVC)
#define SDF_DEPRECATED
#define SDF_FORCEINLINE __forceinline
#else
#define SDF_DEPRECATED
#define SDF_FORCEINLINE
#endif

namespace sdf
{
  /// \brief check if two values are equal, within a tolerance
  /// \param[in] _a the first value
  /// \param[in] _b the second value
  /// \param[in] _epsilon the tolerance
  template<typename T>
  inline bool equal(const T &_a, const T &_b,
                    const T &_epsilon = 1e-6)
  {
    return std::fabs(_a - _b) <= _epsilon;
  }

  class Angle
  {
    /// \brief Stream insertion operator. Outputs in degrees
    /// \param[in] _out output stream
    /// \param[in] _a angle to output
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Angle &_a)
    {
      _out << _a.value;
      return _out;
    }

    /// \brief Stream extraction operator. Assumes input is in degrees
    /// \param in input stream
    /// \param pt angle to read value into
    /// \return The input stream
    public: friend std::istream &operator>>(std::istream &_in,
                                            Angle &_a)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _a.value;
      return _in;
    }

    /// The angle in radians
    private: double value;
  };

  class Color
  {
    public: Color(double _r = 0.0, double _g = 0.0,
                double _b = 0.0, double _a = 1.0)
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


    public: double r;
    public: double g;
    public: double b;
    public: double a;
  };

  class Vector2i
  {
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
    /// \param[in] _pt Vector3 to read values into
    /// \return The stream
    public: friend std::istream &operator>>(std::istream &_in,
                Vector2i &_pt)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _pt.x >> _pt.y;
      return _in;
    }
    public: int x;
    public: int y;
  };

  class Vector2d
  {
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
    /// \param[in] _pt Vector3 to read values into
    /// \return The stream
    public: friend std::istream &operator>>(std::istream &_in,
                Vector2d &_pt)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _pt.x >> _pt.y;
      return _in;
    }

    public: double x;
    public: double y;
  };

  class Vector3
  {
    public: Vector3(const Vector3 &_v)
            : x(_v.x), y(_v.y), z(_v.z) {}

    public: Vector3(double _x = 0.0, double _y = 0.0, double _z = 0.0)
            : x(_x), y(_y), z(_z) {}

    /// \brief Addition operator
    /// \param[in] _v vector to add
    /// \return the sum vector
    public: Vector3 operator+(const Vector3 &_v) const
    {
      return Vector3(this->x + _v.x, this->y + _v.y, this->z + _v.z);
    }

    public: Vector3 Cross(const Vector3 &_pt) const
            {
              Vector3 c(0, 0, 0);

              c.x = this->y * _pt.z - this->z * _pt.y;
              c.y = this->z * _pt.x - this->x * _pt.z;
              c.z = this->x * _pt.y - this->y * _pt.x;

              return c;
            }
    public: Vector3 operator*(double v) const
            {
              return Vector3(this->x * v, this->y * v, this->z * v);
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

    public: const Vector3 &operator*=(double v)
            {
              this->x *= v;
              this->y *= v;
              this->z *= v;

              return *this;
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

    public: double x;
    public: double y;
    public: double z;
  };

  class Vector4
  {
    public: Vector4(double _x = 0.0, double _y = 0.0,
                double _z = 0.0, double _w = 0.0)
            : x(_x), y(_y), z(_z), w(_w) {}
    public: double x;
    public: double y;
    public: double z;
    public: double w;
  };

  class Quaternion
  {
    /// \brief Copy constructor
    /// \param[in] _q Quaternion to copy
    public: Quaternion(const Quaternion &_q)
            : x(_q.x), y(_q.y), z(_q.z), w(_q.w) {}

    /// \brief Constructor
    /// \param[in] _w W param
    /// \param[in] _x X param
    /// \param[in] _y Y param
    /// \param[in] _z Z param
    public: Quaternion(double _w = 1.0, double _x = 0.0, double _y = 0.0,
                       double _z = 0.0)
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
      vec.x = atan2(2 * (copy.y*copy.z + copy.w*copy.x), squ - sqx - sqy + sqz);

      // Pitch
      double sarg = -2 * (copy.x*copy.z - copy.w * copy.y);
      vec.y = sarg <= -1.0 ? -0.5*M_PI : (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));

      // Yaw
      vec.z = atan2(2 * (copy.x*copy.y + copy.w*copy.z), squ + sqx - sqy - sqz);

      return vec;
    }

    public: void SetFromEuler(const Vector3 &_vec)
    {
      double phi, the, psi;

      phi = _vec.x / 2.0;
      the = _vec.y / 2.0;
      psi = _vec.z / 2.0;

      this->w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
      this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
      this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
      this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

      this->Normalize();
    }

    public: void Normalize()
    {
      double s = 0;

      s = sqrt(this->w * this->w + this->x * this->x + this->y * this->y +
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

    public: Vector3 operator*(const Vector3 &v) const
            {
              Vector3 uv, uuv;
              Vector3 qvec(this->x, this->y, this->z);
              uv = qvec.Cross(v);
              uuv = qvec.Cross(uv);
              uv *= (2.0f * this->w);
              uuv *= 2.0f;

              return v + uv + uuv;
            }

    /// \brief Stream extraction operator
    /// \param[in] _in input stream
    /// \param[in] _q Quaternion to read values into
    /// \return The istream
    public: friend std::istream &operator>>(std::istream &_in,
                                             Quaternion &_q)
    {
      double r, p, y;

      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> r >> p >> y;

      _q.SetFromEuler(Vector3(r, p, y));

      return _in;
    }

    public: double x;
    public: double y;
    public: double z;
    public: double w;
  };

  class Pose
  {
    public: Pose()
            {}

    public: Pose(Vector3 _pos, Quaternion _rot)
            : pos(_pos), rot(_rot) {}


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

    public: Pose operator*(const Pose &pose)
            {
              return Pose(this->CoordPositionAdd(pose),  pose.rot * this->rot);
            }

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

    public: Vector3 pos;
    public: Quaternion rot;
  };

  class Time
  {
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

    public: int32_t sec;
    public: int32_t nsec;
  };

  class Mass
  {
    public: double mass;
  };
}
#endif
