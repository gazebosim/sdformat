/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef USD_MODEL_TYPES_H
#define USD_MODEL_TYPES_H

#include <memory>

#include <string>
#include <stdexcept>

namespace usd
{

inline
void split_string(std::vector<std::string> &result,
                  const std::string &input,
                  const std::string &isAnyOf)
{
  std::string::size_type start = 0;
  std::string::size_type end = input.find_first_of(isAnyOf, start);
  while (end != std::string::npos)
  {
    result.push_back(input.substr(start, end-start));
    start = end + 1;
    end = input.find_first_of(isAnyOf, start);
  }
  if (start < input.length())
  {
    result.push_back(input.substr(start));
  }
}

class ParseError: public std::runtime_error
{
public:
  ParseError(const std::string &error_msg) : std::runtime_error(error_msg) {};
};

}

#define USD_TYPEDEF_CLASS_POINTER(Class) \
class Class; \
typedef std::shared_ptr<Class> Class##SharedPtr; \
typedef std::shared_ptr<const Class> Class##ConstSharedPtr; \
typedef std::weak_ptr<Class> Class##WeakPtr


namespace usd{

  // shared pointer used in joint.h
  typedef std::shared_ptr<double> DoubleSharedPtr;

  class ModelInterface;

  USD_TYPEDEF_CLASS_POINTER(Box);
  USD_TYPEDEF_CLASS_POINTER(Collision);
  USD_TYPEDEF_CLASS_POINTER(Cylinder);
  USD_TYPEDEF_CLASS_POINTER(Geometry);
  USD_TYPEDEF_CLASS_POINTER(Inertial);
  USD_TYPEDEF_CLASS_POINTER(Joint);
  USD_TYPEDEF_CLASS_POINTER(JointCalibration);
  USD_TYPEDEF_CLASS_POINTER(JointDynamics);
  USD_TYPEDEF_CLASS_POINTER(JointLimits);
  USD_TYPEDEF_CLASS_POINTER(JointMimic);
  USD_TYPEDEF_CLASS_POINTER(JointSafety);
  USD_TYPEDEF_CLASS_POINTER(Link);
  USD_TYPEDEF_CLASS_POINTER(Material);
  USD_TYPEDEF_CLASS_POINTER(Sphere);
  USD_TYPEDEF_CLASS_POINTER(Visual);
  // typedef shared pointers
  typedef std::shared_ptr<ModelInterface> ModelInterfaceSharedPtr;

  // create *_pointer_cast functions in urdf namespace
  template<class T, class U>
  std::shared_ptr<T> const_pointer_cast(std::shared_ptr<U> const & r)
  {
    return std::const_pointer_cast<T>(r);
  }

  template<class T, class U>
  std::shared_ptr<T> dynamic_pointer_cast(std::shared_ptr<U> const & r)
  {
    return std::dynamic_pointer_cast<T>(r);
  }

  template<class T, class U>
  std::shared_ptr<T> static_pointer_cast(std::shared_ptr<U> const & r)
  {
    return std::static_pointer_cast<T>(r);
  }


}

#endif
