/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include <stdio.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include "sdf/Param.hh"

int check_double(std::string num)
{
    const std::string name = "number";
    const std::string type = "double";
    const std::string def = "0.0";
    
    sdf::Param param(name, type, def, true);
    return param.SetFromString(num);
}

TEST(SetFromString, Decimals)
{
  ASSERT_TRUE(check_double("0.2345"));
}
