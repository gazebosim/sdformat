/*
 * Copyright 2020 Open Source Robotics Foundation
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
#include <sstream>
#include <gtest/gtest.h>

#include "ParamPassing.hh"
#include "sdf/Element.hh"
#include "sdf/parser.hh"

/////////////////////////////////////////////////
TEST(ParamPassing, GetElement)
{
  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.7'>"
         << "  <model name='test'>"
         << "    <model name='test_model'>"
         << "      <link name='test_link'/>"
         << "    </model>"
         << "  </model>"
         << "</sdf>";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  ASSERT_TRUE(sdf::readString(stream.str(), sdf));

  sdf::ElementPtr elemLink = sdf->Root()->GetFirstElement()
                                        ->GetElement("model")
                                        ->GetElement("link");
  EXPECT_NE(nullptr, elemLink);

  sdf::ElementPtr paramPassLink = getElementById(sdf,
                                                 "test_model::test_link",
                                                 "link",
                                                 false);
  EXPECT_NE(nullptr, paramPassLink);
  EXPECT_EQ(elemLink, paramPassLink);
}
