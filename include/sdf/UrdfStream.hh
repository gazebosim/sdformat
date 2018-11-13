/*
 * Copyright 2018 Open Source Robotics Foundation
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
#ifndef SDF_URDFSTREAM_HH_
#define SDF_URDFSTREAM_HH_

#include <string>
#include <iostream>

#include <sdf/Model.hh>

namespace sdf
{
  class SDFORMAT_VISIBLE UrdfStream
  {
    public: UrdfStream(const sdf::Model &_model);

    public: std::string String() const;

    /// \brief Stream insertion operator
    /// \param _out output stream
    /// \param _stream UrdfStream to output
    /// \return the stream
    public: friend std::ostream &operator<<(
                std::ostream &_out, const UrdfStream &_stream)
    {
      _out << _stream.String();
      return _out;
    }
  };
}

#endif
