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

#ifndef USD_MODEL_COLOR_HH
#define USD_MODEL_COLOR_HH

#include <stdexcept>
#include <string>
#include <vector>
#include <math.h>

#include "usd_model/types.hh"

namespace usd
{

class Color
{
public:
  Color() {this->clear();};
  float r;
  float g;
  float b;
  float a;

  void clear()
  {
    r = g = b = 0.0f;
    a = 1.0f;
  }
  bool init(const std::string &vector_str)
  {
    this->clear();
    std::vector<std::string> pieces;
    std::vector<float> rgba;
    usd::split_string( pieces, vector_str, " ");
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (!pieces[i].empty())
      {
        try
        {
          rgba.push_back(std::stof(pieces[i]));
        }
        catch (std::invalid_argument &/*e*/) {
          return false;
        }
        catch (std::out_of_range &/*e*/) {
          return false;
        }
      }
    }

    if (rgba.size() != 4)
    {
      return false;
    }

    this->r = rgba[0];
    this->g = rgba[1];
    this->b = rgba[2];
    this->a = rgba[3];

    return true;
  };
};

}


#endif
