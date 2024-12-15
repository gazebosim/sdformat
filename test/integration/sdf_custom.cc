/*
 * Copyright 2019 Open Source Robotics Foundation
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

#include <string>

#include <gtest/gtest.h>

#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

#include "test_config.hh"

/////////////////////////////////////////////////
TEST(SDFParser, CustomElements)
{
  const std::string sdfTestFile =
      sdf::testing::TestFile("integration", "custom_elems_attrs.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(sdfTestFile).empty());

  const sdf::World *world = root.WorldByIndex(0);

  std::string worldName = world->Name();
  EXPECT_EQ("W", worldName);

  // Use of sdf::World::Element() to obtain an sdf::ElementPtr object
  sdf::ElementPtr worldElement = world->Element();

  // Use of sdf::ElementPtr::GetAttribute()
  sdf::ParamPtr typeParam = worldElement->GetAttribute("mysim:type");
  std::string simType;
  // Use of sdf::ParamPtr::Get<T>()
  typeParam->Get<std::string>(simType);
  EXPECT_EQ("2d", simType);

  const sdf::Model *model = world->ModelByIndex(0);

  const sdf::Link *link1 = model->LinkByIndex(0);
  // Use of sdf::Link::Element() to obtain an sdf::ElementPtr object
  sdf::ElementPtr link1Element = link1->Element();

  // Use of sdf::ElementPtr::Get<T>() to obtain the value of an attribute
  auto customAttrStr = link1Element->Get<std::string>("mysim:custom_attr_str");
  EXPECT_EQ("A", customAttrStr);

  auto customAttrInt = link1Element->Get<int>("mysim:custom_attr_int");
  EXPECT_EQ(5, customAttrInt);

  // Check empty attribute in link L2
  const sdf::Link *link2 = model->LinkByName("L2");
  ASSERT_TRUE(link2 != nullptr);
  sdf::ElementPtr link2Element = link2->Element();
  ASSERT_TRUE(link2Element != nullptr);
  EXPECT_TRUE(link2Element->HasAttribute("mysim:empty_attr"));
  auto emptyAttrStr = link2Element->Get<std::string>("mysim:empty_attr");
  EXPECT_EQ("", emptyAttrStr);

  // Ensure that mysim:empty_attr appears when calling ToString("")
  auto rootToString = link2Element->ToString("");
  EXPECT_NE(std::string::npos, rootToString.find("mysim:empty_attr=''"))
    << rootToString;

  // Use of sdf::Model::Element() to obtain an sdf::ElementPtr object
  sdf::ElementPtr modelElement = model->Element();

  sdf::ElementPtr transmission = modelElement->GetElement("mysim:transmission");
  auto transmissionName = transmission->Get<std::string>("name");
  EXPECT_EQ("simple_trans", transmissionName);

  auto transmissionType = transmission->Get<std::string>("mysim:type");
  EXPECT_EQ("transmission_interface/SimpleTransmission", transmissionType);

  sdf::ElementPtr tranJointElement = transmission->GetElement("mysim:joint");
  auto tranJointName = tranJointElement->Get<std::string>("name");
  EXPECT_EQ("J1", tranJointName);

  sdf::ElementPtr transHwInterfaceElement =
      tranJointElement->GetElement("mysim:hardwareInterface");

  // Use of sdf::ElementPtr::Get<T>() to obtain the value of a child element
  auto tranHwInterface =
      tranJointElement->Get<std::string>("mysim:hardwareInterface");
  EXPECT_EQ("EffortJointInterface", tranHwInterface);
}

/////////////////////////////////////////////////
TEST(SDFParser, ReloadCustomElements)
{
  const std::string sdfTestFile =
      sdf::testing::TestFile("integration", "custom_elems_attrs.sdf");

  // load file with custom elements
  sdf::Root root1;
  sdf::Errors errors = root1.Load(sdfTestFile);
  EXPECT_TRUE(errors.empty());

  // reload string output of root1
  sdf::Root root2;
  errors = root2.LoadSdfString(root1.Element()->ToString(""));
  EXPECT_TRUE(errors.empty());

  // check that root1 and root2 equal
  const sdf::World *world1 = root1.WorldByIndex(0);
  const sdf::World *world2 = root2.WorldByIndex(0);
  ASSERT_NE(nullptr, world1);
  ASSERT_NE(nullptr, world2);

  const sdf::Model *model1 = world1->ModelByIndex(0);
  const sdf::Model *model2 = world2->ModelByIndex(0);
  ASSERT_NE(nullptr, model1);
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ(model1->Element()->ToString(""), model2->Element()->ToString(""));

  const sdf::Link *link1 = model1->LinkByIndex(0);
  const sdf::Link *link2 = model2->LinkByIndex(0);
  ASSERT_NE(nullptr, link1);
  ASSERT_NE(nullptr, link2);
  EXPECT_EQ(link1->Element()->ToString(""), link2->Element()->ToString(""));

  sdf::ElementPtr customElem1 =
      model1->Element()->FindElement("mysim:transmission");
  sdf::ElementPtr customElem2 =
      model2->Element()->FindElement("mysim:transmission");
  ASSERT_NE(nullptr, customElem1);
  ASSERT_NE(nullptr, customElem2);

  const std::string customElemStr =
R"(<mysim:transmission name='simple_trans' mysim:attr='custom_attribute'>
  <mysim:type>transmission_interface/SimpleTransmission</mysim:type>
  <mysim:joint name='J1'>
    <mysim:hardwareInterface>EffortJointInterface</mysim:hardwareInterface>
  </mysim:joint>
</mysim:transmission>
)";
  EXPECT_EQ(customElemStr, customElem1->ToString(""));
  EXPECT_EQ(customElemStr, customElem2->ToString(""));

  sdf::ElementPtr customDesc1 =
      world1->Element()->FindElement("mysim:description");
  sdf::ElementPtr customDesc2 =
      world2->Element()->FindElement("mysim:description");
  ASSERT_NE(nullptr, customDesc1);
  ASSERT_NE(nullptr, customDesc2);

  const std::string customDescStr =
    "<mysim:description>Description of this world</mysim:description>\n";
  EXPECT_EQ(customDescStr, customDesc1->ToString(""));
  EXPECT_EQ(customDescStr, customDesc2->ToString(""));
}

/////////////////////////////////////////////////
TEST(SDFParser, ReloadIncludedCustomElements)
{
  const std::string modelPath = sdf::testing::TestFile("integration", "model");

  sdf::setFindCallback(
    [&](const std::string &_file)
    {
      return sdf::filesystem::append(modelPath, _file);
    });

  const std::string sdfStr =
R"(<sdf version='1.7'>
  <world name='default'>
    <include>
      <uri>model_with_custom_elements</uri>
    </include>
  </world>
</sdf>
)";

  // load included file with custom elements
  sdf::Root root1;
  sdf::Errors errors = root1.LoadSdfString(sdfStr);
  EXPECT_TRUE(errors.empty());

  // reload string output of root1
  sdf::Root root2;
  errors = root2.LoadSdfString(root1.Element()->ToString(""));
  EXPECT_TRUE(errors.empty());

  // check that root1 and root2 equal
  EXPECT_EQ(root1.Element()->ToString(""), root2.Element()->ToString(""));

  const sdf::World *world1 = root1.WorldByIndex(0);
  const sdf::World *world2 = root2.WorldByIndex(0);
  ASSERT_NE(nullptr, world1);
  ASSERT_NE(nullptr, world2);

  // //model[@name=M1]
  const sdf::Model *model11 = world1->ModelByIndex(0);
  const sdf::Model *model12 = world2->ModelByIndex(0);
  ASSERT_NE(nullptr, model11);
  ASSERT_NE(nullptr, model12);
  EXPECT_EQ(model11->Element()->ToString(""), model12->Element()->ToString(""));

  // //model[@name=M1]/link[@name=L1]
  const sdf::Link *model11link1 = model11->LinkByIndex(0);
  const sdf::Link *model12link2 = model12->LinkByIndex(0);
  ASSERT_NE(nullptr, model11link1);
  ASSERT_NE(nullptr, model12link2);

  const std::string linkCustomAttrStr =
    "<link name='L1' mysim:custom_attr_str='A' mysim:custom_attr_int='5'/>\n";
  EXPECT_EQ(linkCustomAttrStr, model11link1->Element()->ToString(""));
  EXPECT_EQ(linkCustomAttrStr, model12link2->Element()->ToString(""));

  // //model[@name=M1]/model[@name=M2]
  const sdf::Model *model21 = model11->ModelByIndex(0);
  const sdf::Model *model22 = model12->ModelByIndex(0);
  ASSERT_NE(nullptr, model21);
  ASSERT_NE(nullptr, model22);
  EXPECT_EQ(model21->Element()->ToString(""), model22->Element()->ToString(""));

  // //model[@name=M1]/model[@name=M2]/link[@name=L1]
  const sdf::Link *model21link1 = model21->LinkByIndex(0);
  const sdf::Link *model22link2 = model22->LinkByIndex(0);
  ASSERT_NE(nullptr, model21link1);
  ASSERT_NE(nullptr, model22link2);
  EXPECT_EQ(model21link1->Element()->ToString(""),
            model22link2->Element()->ToString(""));

  // check custom attributes
  sdf::ParamPtr param1 =
      model21link1->Element()->GetAttribute("mysim:custom_attr_str");
  sdf::ParamPtr param2 =
      model22link2->Element()->GetAttribute("mysim:custom_attr_str");
  ASSERT_NE(nullptr, param1);
  ASSERT_NE(nullptr, param2);
  EXPECT_EQ("B", param1->GetAsString());
  EXPECT_EQ("B", param2->GetAsString());

  // //model[@name=M1]/model[@name=M2]/link[@name=L1]/mysim:transmission
  sdf::ElementPtr customElem1 =
      model21link1->Element()->FindElement("mysim:transmission");
  sdf::ElementPtr customElem2 =
      model22link2->Element()->FindElement("mysim:transmission");
  ASSERT_NE(nullptr, customElem1);
  ASSERT_NE(nullptr, customElem2);

  const std::string customElemStr =
R"(<mysim:transmission name='simple_trans'>
  <mysim:type>transmission_interface/SimpleTransmission</mysim:type>
  <mysim:joint name='J1'>
    <mysim:hardwareInterface>EffortJointInterface</mysim:hardwareInterface>
  </mysim:joint>
</mysim:transmission>
)";
  EXPECT_EQ(customElemStr, customElem1->ToString(""));
  EXPECT_EQ(customElemStr, customElem2->ToString(""));
}

/////////////////////////////////////////////////
TEST(SDFParser, ReloadNestedIncludedCustomElements)
{
  const std::string modelPath = sdf::testing::TestFile("integration", "model");

  sdf::setFindCallback(
    [&](const std::string &_file)
    {
      return sdf::filesystem::append(modelPath, _file);
    });

  const std::string sdfStr =
R"(<sdf version='1.7'>
  <world name='default'>
    <model name='test'>
      <include>
        <uri>model_with_custom_elements</uri>
      </include>
    </model>
  </world>
</sdf>
)";

  sdf::Root root1;
  sdf::Errors errors = root1.LoadSdfString(sdfStr);
  EXPECT_TRUE(errors.empty());

  for (auto &e : errors)
    std::cout << e.Message() << std::endl;

  sdf::Root root2;
  errors = root2.LoadSdfString(root1.Element()->ToString(""));
  EXPECT_TRUE(errors.empty());

  // check that root1 and root2 equal
  EXPECT_EQ(root1.Element()->ToString(""), root2.Element()->ToString(""));

  const sdf::World *world1 = root1.WorldByIndex(0);
  const sdf::World *world2 = root2.WorldByIndex(0);
  ASSERT_NE(nullptr, world1);
  ASSERT_NE(nullptr, world2);

  // //model[@name=test]
  const sdf::Model *model01 = world1->ModelByIndex(0);
  const sdf::Model *model02 = world2->ModelByIndex(0);
  ASSERT_NE(nullptr, model01);
  ASSERT_NE(nullptr, model02);
  EXPECT_EQ("test", model01->Name());
  EXPECT_EQ("test", model02->Name());
  EXPECT_EQ(model01->Element()->ToString(""), model02->Element()->ToString(""));

  // //model[@name=test]/model[@name=M1]
  const sdf::Model *model11 = model01->ModelByIndex(0);
  const sdf::Model *model12 = model02->ModelByIndex(0);
  ASSERT_NE(nullptr, model11);
  ASSERT_NE(nullptr, model12);
  EXPECT_EQ("M1", model11->Name());
  EXPECT_EQ("M1", model12->Name());
  EXPECT_EQ(model11->Element()->ToString(""), model12->Element()->ToString(""));

  // //model[@name=test]/model[@name=M1]/link[@name=L1]
  const sdf::Link *model11link1 = model11->LinkByIndex(0);
  const sdf::Link *model12link2 = model12->LinkByIndex(0);
  ASSERT_NE(nullptr, model11link1);
  ASSERT_NE(nullptr, model12link2);
  EXPECT_EQ("L1", model11link1->Name());
  EXPECT_EQ("L1", model12link2->Name());

  const std::string linkCustomAttrStr =
R"(<link name='L1' mysim:custom_attr_str='A' mysim:custom_attr_int='5'/>
)";
  EXPECT_EQ(linkCustomAttrStr, model11link1->Element()->ToString(""));
  EXPECT_EQ(linkCustomAttrStr, model12link2->Element()->ToString(""));

  // //model[@name=test]/model[@name=M1]/model[@name=M2]
  const sdf::Model *model21 = model11->ModelByIndex(0);
  const sdf::Model *model22 = model12->ModelByIndex(0);
  ASSERT_NE(nullptr, model21);
  ASSERT_NE(nullptr, model22);
  EXPECT_EQ("M2", model21->Name());
  EXPECT_EQ("M2", model22->Name());
  EXPECT_EQ(model21->Element()->ToString(""), model22->Element()->ToString(""));

  // //model[@name=test]/model[@name=M1]/model[@name=M2]/link[@name=L1]
  const sdf::Link *model21link1 = model21->LinkByIndex(0);
  const sdf::Link *model22link2 = model22->LinkByIndex(0);
  ASSERT_NE(nullptr, model21link1);
  ASSERT_NE(nullptr, model22link2);
  EXPECT_EQ("L1", model21link1->Name());
  EXPECT_EQ("L1", model22link2->Name());
  EXPECT_EQ(model21link1->Element()->ToString(""),
            model22link2->Element()->ToString(""));

  // check custom attributes
  sdf::ParamPtr param1 =
      model21link1->Element()->GetAttribute("mysim:custom_attr_str");
  sdf::ParamPtr param2 =
      model22link2->Element()->GetAttribute("mysim:custom_attr_str");
  ASSERT_NE(nullptr, param1);
  ASSERT_NE(nullptr, param2);
  EXPECT_EQ("B", param1->GetAsString());
  EXPECT_EQ("B", param2->GetAsString());

  // //model[@name=test]/model[@name=M1::M2]/link[@name=M1::L1]
  //  /mysim:transmission
  sdf::ElementPtr customElem1 =
      model21link1->Element()->FindElement("mysim:transmission");
  sdf::ElementPtr customElem2 =
      model22link2->Element()->FindElement("mysim:transmission");
  ASSERT_NE(nullptr, customElem1);
  ASSERT_NE(nullptr, customElem2);

  const std::string customElemStr =
R"(<mysim:transmission name='simple_trans'>
  <mysim:type>transmission_interface/SimpleTransmission</mysim:type>
  <mysim:joint name='J1'>
    <mysim:hardwareInterface>EffortJointInterface</mysim:hardwareInterface>
  </mysim:joint>
</mysim:transmission>
)";
  EXPECT_EQ(customElemStr, customElem1->ToString(""));
  EXPECT_EQ(customElemStr, customElem2->ToString(""));
}
