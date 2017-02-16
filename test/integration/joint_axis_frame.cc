/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <string>
#include "sdf/sdf.hh"

#include "test_config.h"

std::string get_sdf_string(std::string _version)
{
  std::ostringstream stream;
  stream
    << "<sdf version='" << _version << "'>"
    << "<model name='model'>"
    << "  <link name='parent'/>"
    << "  <link name='child'/>"
    << "  <joint name='joint' type='revolute'>"
    << "    <parent>parent</parent>"
    << "    <child>child</child>"
    << "    <axis>"
    << "      <xyz>1 0 0</xyz>"
    << "    </axis>"
    << "  </joint>"
    << "</model>"
    << "</sdf>";
  return stream.str();
}

////////////////////////////////////////
// sdf model, version 1.4, use_parent_model_frame tag missing
// expect tag to be inserted with value true
TEST(JointAxisFrame, Version_1_4_missing)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string("1.4"), model));

  sdf::ElementPtr joint = model->Root()->GetElement(
    "model")->GetElement("joint");
  sdf::ElementPtr axis = joint->GetElement("axis");

  axis->PrintValues("");
  EXPECT_TRUE(axis->HasElement("use_parent_model_frame"));
  EXPECT_TRUE(axis->Get<bool>("use_parent_model_frame"));
}

////////////////////////////////////////
// sdf model, version 1.5, use_parent_model_frame tag missing
// expect tag to be inserted with value false
TEST(JointAxisFrame, Version_1_5_missing)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string("1.5"), model));

  sdf::ElementPtr joint = model->Root()->GetElement(
    "model")->GetElement("joint");
  sdf::ElementPtr axis = joint->GetElement("axis");
  EXPECT_TRUE(axis->HasElement("use_parent_model_frame"));
  EXPECT_FALSE(axis->Get<bool>("use_parent_model_frame"));
}
