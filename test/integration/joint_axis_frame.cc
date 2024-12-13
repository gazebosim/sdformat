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

#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "sdf/parser.hh"
#include "sdf/Root.hh"

std::string get_sdf_string(const std::string &_version)
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
// sdf model, version 1.4, use_parent_model_frame tag should
// be removed when converted to 1.7.
TEST(JointAxisFrame, Version_1_4_missing)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string("1.4"), model));

  sdf::ElementPtr joint = model->Root()->GetElement(
    "model")->GetElement("joint");
  sdf::ElementPtr axis = joint->GetElement("axis");
  ASSERT_NE(nullptr, axis);

  axis->PrintValues("");
  EXPECT_FALSE(axis->HasElement("use_parent_model_frame"));

  sdf::ElementPtr xyz = axis->GetElement("xyz");
  ASSERT_NE(nullptr, xyz);
  ASSERT_TRUE(xyz->HasAttribute("expressed_in"));
  EXPECT_EQ("__model__", xyz->Get<std::string>("expressed_in"));

  // Try to load DOM object and expect it to succeed with no errors
  sdf::Root root;
  auto errors = root.Load(model);
  EXPECT_EQ(0u, errors.size());
}

////////////////////////////////////////
// sdf model, version 1.4, use_parent_model_frame tag should
// not be added when reading without converting.
TEST(JointAxisFrame, Version_1_4_no_convert)
{
  sdf::Errors errors;
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(
      sdf::readStringWithoutConversion(get_sdf_string("1.4"), model, errors));

  EXPECT_EQ("1.4", model->Root()->Get<std::string>("version"));
  sdf::ElementPtr joint = model->Root()->GetElement(
    "model")->GetElement("joint");
  sdf::ElementPtr axis = joint->GetElement("axis");
  ASSERT_NE(nullptr, axis);

  axis->PrintValues("");
  EXPECT_FALSE(axis->HasElement("use_parent_model_frame"));

  // Try to load DOM object and expect it to fail since SDF is not updated
  sdf::Root root;
  errors = root.Load(model);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  const auto expErr =
      "SDF version attribute[1.4] should match the latest "
      "version[" SDF_PROTOCOL_VERSION "] when loading DOM objects.";
  EXPECT_TRUE(errors[0].Message().find(expErr) != std::string::npos)
      << errors[0].Message();
}

////////////////////////////////////////
// sdf model, version 1.4, use_parent_model_frame tag should
// be added when converted to 1.5.
TEST(JointAxisFrame, Version_1_4_to_1_5)
{
  sdf::Errors errors;
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::convertString(get_sdf_string("1.4"), "1.5", model));

  EXPECT_EQ("1.5", model->Root()->Get<std::string>("version"));
  sdf::ElementPtr joint = model->Root()->GetElement(
    "model")->GetElement("joint");
  sdf::ElementPtr axis = joint->GetElement("axis");
  ASSERT_NE(nullptr, axis);

  axis->PrintValues("");
  EXPECT_TRUE(axis->HasElement("use_parent_model_frame"));
  EXPECT_TRUE(axis->Get<bool>("use_parent_model_frame"));

  // Try to load DOM object and expect it to fail since SDF is not updated
  sdf::Root root;
  errors = root.Load(model);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_INVALID);
  const auto expErr =
      "SDF version attribute[1.5] should match the latest "
      "version[" SDF_PROTOCOL_VERSION "] when loading DOM objects.";
  EXPECT_TRUE(errors[0].Message().find(expErr) != std::string::npos)
      << errors[0].Message();
}

////////////////////////////////////////
// sdf model, version 1.5, use_parent_model_frame tag should
// be removed when converted to 1.7.
TEST(JointAxisFrame, Version_1_5_missing)
{
  sdf::SDFPtr model(new sdf::SDF());
  sdf::init(model);
  ASSERT_TRUE(sdf::readString(get_sdf_string("1.5"), model));

  sdf::ElementPtr joint = model->Root()->GetElement(
    "model")->GetElement("joint");
  sdf::ElementPtr axis = joint->GetElement("axis");
  EXPECT_FALSE(axis->HasElement("use_parent_model_frame"));

  // Try to load DOM object and expect it to succeed with no errors
  sdf::Root root;
  auto errors = root.Load(model);
  EXPECT_EQ(0u, errors.size());
}
