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
         << "      <link name='test_link'>"
         << "        <collision name='test_visual'>"
         << "          <geometry>"
         << "            <box>"
         << "              <size>0.1 0.1 0.1</size>"
         << "            </box>"
         << "          </geometry>"
         << "        </collision>"
         << "      </link>"
         << "      <link name='test_link2'>"
         << "        <visual name='test_visual'>"
         << "          <geometry>"
         << "            <box>"
         << "              <size>0.5 0.5 0.5</size>"
         << "            </box>"
         << "          </geometry>"
         << "        </visual>"
         << "      </link>"
         << "    </model>"
         << "  </model>"
         << "</sdf>";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::Errors errors;
  sdf::init(sdf, errors);
  ASSERT_TRUE(errors.empty());
  ASSERT_TRUE(sdf::readString(stream.str(), sdf));

  // checking element ptrs to <link name='test_link'> are equal
  sdf::ElementPtr elem = sdf->Root()->GetFirstElement()
                                        ->GetElement("model")
                                        ->GetElement("link");
  sdf::ElementPtr paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                                 "link",
                                                 "test_model::test_link");
  EXPECT_NE(nullptr, elem);
  EXPECT_NE(nullptr, paramPassElem);
  EXPECT_EQ(elem, paramPassElem);

  // checking element ptrs to <visual name='test_visual'> are equal
  elem = sdf->Root()->GetFirstElement()
                        ->GetElement("model")
                        ->GetElement("link")
                        ->GetNextElement()
                        ->GetElement("visual");
  paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                 "visual",
                                 "test_model::test_link2::test_visual");
  EXPECT_NE(nullptr, elem);
  EXPECT_NE(nullptr, paramPassElem);
  EXPECT_EQ(elem, paramPassElem);

  // No element <visual name='test_visual'> (element is a collision)
  paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                 "visual",
                                 "test_model::test_link::test_visual");
  EXPECT_EQ(nullptr, paramPassElem);

  // incorrect element identifier (model::test_link::test_visual)
  paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                 "collision",
                                 "model::test_link::test_visual");
  EXPECT_EQ(nullptr, paramPassElem);
}
