/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <boost/filesystem.hpp>
#include <boost/any.hpp>
#include "test_config.h"
#include "sdf/sdf.hh"

////////////////////////////////////////////////////
// Testing fixture
class RmlUpdate : public testing::Test
{
  protected: RmlUpdate()
             {
               boost::filesystem::path path =
                 boost::filesystem::path(PROJECT_SOURCE_PATH)
                 / "sdf" / SDF_VERSION;

               // Store original env var.
               this->origSDFPath = getenv("SDF_PATH");

               setenv("SDF_PATH", path.string().c_str(), 1);
             }

  protected: virtual void TearDown()
             {
               // Restore original env var.
               // osx segfaults unless this check is in place
               // some discussion of portability of setenv at:
               // http://www.greenend.org.uk/rjk/tech/putenv.html
               if (this->origSDFPath)
                 setenv("SDF_PATH", this->origSDFPath, 1);
             }

  private: char *origSDFPath;
};

class RmlUpdateFixture
{
  public:  std::string GetName() const {return this->name;}
  public:  bool GetFlag() const {return this->flag;}
  public:  sdf::Pose GetPose() const {return this->pose;}
  public:  std::string name;
  public:  bool flag;
  public:  sdf::Pose pose;
};

////////////////////////////////////////////////////
/// Ensure that SDF::Update is working for attributes
TEST_F(RmlUpdate, UpdateAttribute)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='test_model'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.root->GetElement("model");

  // Read name attribute value
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  sdf::ParamPtr nameParam = modelElem->GetAttribute("name");
  EXPECT_EQ(nameParam->GetType(), typeid(std::string));

  // Set test class variables based on sdf values
  // Set parameter update functions to test class accessors
  RmlUpdateFixture fixture;
  nameParam->Get(fixture.name);
  nameParam->SetUpdateFunc(boost::bind(&RmlUpdateFixture::GetName, &fixture));

  std::string nameCheck;
  int i;
  for (i = 0; i < 4; i++)
  {
    // Update test class variables
    fixture.name[0] = 'd' + i;

    // Update root sdf element
    sdfParsed.root->Update();

    // Expect sdf values to match test class variables
    nameParam->Get(nameCheck);
    EXPECT_STREQ(nameCheck.c_str(), fixture.name.c_str());
  }
}

////////////////////////////////////////////////////
/// Ensure that SDF::Update is working for elements
TEST_F(RmlUpdate, UpdateElement)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='test_model'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.root->GetElement("model");

  // Read static element value
  EXPECT_TRUE(modelElem->HasElement("static"));
  sdf::ParamPtr staticParam = modelElem->GetElement("static")->GetValue();
  EXPECT_TRUE(staticParam->GetType() == typeid(bool));

  // Read pose element value
  EXPECT_TRUE(modelElem->HasElement("pose"));
  sdf::ParamPtr poseParam = modelElem->GetElement("pose")->GetValue();
  EXPECT_TRUE(poseParam->GetType() == typeid(sdf::Pose));

  // Set test class variables based on sdf values
  // Set parameter update functions to test class accessors
  RmlUpdateFixture fixture;
  staticParam->Get(fixture.flag);
  staticParam->SetUpdateFunc(boost::bind(&RmlUpdateFixture::GetFlag, &fixture));
  poseParam->Get(fixture.pose);
  poseParam->SetUpdateFunc(boost::bind(&RmlUpdateFixture::GetPose, &fixture));

  bool flagCheck;
  sdf::Pose poseCheck;
  int i;
  for (i = 0; i < 4; i++)
  {
    // Update test class variables
    fixture.flag = !fixture.flag;
    fixture.pose.pos.x = i;
    fixture.pose.pos.y = i+10;
    fixture.pose.pos.z = -i*i*i;

    // Update root sdf element
    sdfParsed.root->Update();

    // Expect sdf values to match test class variables
    staticParam->Get(flagCheck);
    EXPECT_EQ(flagCheck, fixture.flag);
    poseParam->Get(poseCheck);
    EXPECT_EQ(poseCheck, fixture.pose);
  }
}

////////////////////////////////////////////////////
/// Ensure that SDF::Element::RemoveFromParent is working
TEST_F(RmlUpdate, ElementRemoveFromParent)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='model1'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model2'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model3'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr elem;

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  elem = sdfParsed.root->GetElement("model");

  // Select the second model named 'model2'
  elem = elem->GetNextElement("model");
  EXPECT_TRUE(elem);
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model2");

  // Remove model2
  elem->RemoveFromParent();

  // Get first model element again
  elem = sdfParsed.root->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model1");

  // Get next model element
  elem = elem->GetNextElement("model");
  // Check name == model3
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model3");

  // Try to get another model element
  elem = elem->GetNextElement("model");
  EXPECT_FALSE(elem);
}

////////////////////////////////////////////////////
/// Ensure that SDF::Element::RemoveChild is working
TEST_F(RmlUpdate, ElementRemoveChild)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='model1'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model2'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model3'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr elem, elem2;

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  elem = sdfParsed.root->GetElement("model");

  // Select the static element in model1
  elem2 = elem->GetElement("static");
  EXPECT_TRUE(elem2);
  EXPECT_FALSE(elem2->Get<bool>());
  elem->RemoveChild(elem2);

  // Get first model element again
  elem = sdfParsed.root->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model1");

  // Check that we have deleted the static element in model1
  EXPECT_FALSE(elem->HasElement("static"));

  // Get model2
  elem2 = elem->GetNextElement("model");

  // Remove model2
  sdfParsed.root->RemoveChild(elem2);

  // Get first model element again
  elem = sdfParsed.root->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model1");

  // Get next model element
  elem = elem->GetNextElement("model");
  // Check name == model3
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model3");

  // Try to get another model element
  elem = elem->GetNextElement("model");
  EXPECT_FALSE(elem);
}

////////////////////////////////////////////////////
/// Ensure that getting empty values with empty keys returns correct values.
TEST_F(RmlUpdate, EmptyValues)
{
  std::string emptyString;
  sdf::ElementPtr elem;

  elem.reset(new sdf::Element());
  EXPECT_FALSE(elem->Get<bool>(emptyString));
  elem->AddValue("bool", "true", "0", "description");
  EXPECT_TRUE(elem->Get<bool>(emptyString));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<int>(emptyString), 0);
  elem->AddValue("int", "12", "0", "description");
  EXPECT_EQ(elem->Get<int>(emptyString), 12);

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<unsigned int>(emptyString),
      static_cast<unsigned int>(0));
  elem->AddValue("unsigned int", "123", "0", "description");
  EXPECT_EQ(elem->Get<unsigned int>(emptyString),
      static_cast<unsigned int>(123));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<char>(emptyString), '\0');
  elem->AddValue("char", "a", "0", "description");
  EXPECT_EQ(elem->Get<char>(emptyString), 'a');

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<std::string>(emptyString), "");
  elem->AddValue("string", "hello", "0", "description");
  EXPECT_EQ(elem->Get<std::string>(emptyString), "hello");

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<sdf::Vector2d>(emptyString), sdf::Vector2d());
  elem->AddValue("vector2d", "1 2", "0", "description");
  EXPECT_EQ(elem->Get<sdf::Vector2d>(emptyString), sdf::Vector2d(1, 2));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<sdf::Vector3>(emptyString), sdf::Vector3());
  elem->AddValue("vector3", "1 2 3", "0", "description");
  EXPECT_EQ(elem->Get<sdf::Vector3>(emptyString), sdf::Vector3(1, 2, 3));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<sdf::Quaternion>(emptyString), sdf::Quaternion());
  elem->AddValue("quaternion", "1 2 3", "0", "description");
  EXPECT_EQ(elem->Get<sdf::Quaternion>(emptyString),
            sdf::Quaternion(-2.14159, 1.14159, -0.141593));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<sdf::Pose>(emptyString), sdf::Pose());
  elem->AddValue("pose", "1 2 3 4 5 6", "0", "description");
  EXPECT_EQ(elem->Get<sdf::Pose>(emptyString), sdf::Pose(1, 2, 3, 4, 5, 6));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<sdf::Color>(emptyString), sdf::Color());
  elem->AddValue("color", ".1 .2 .3 1.0", "0", "description");
  EXPECT_EQ(elem->Get<sdf::Color>(emptyString),
            sdf::Color(.1, .2, .3, 1.0));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<sdf::Time>(emptyString), sdf::Time());
  elem->AddValue("time", "1 2", "0", "description");
  EXPECT_EQ(elem->Get<sdf::Time>(emptyString), sdf::Time(1, 2));

  elem.reset(new sdf::Element());
  EXPECT_NEAR(elem->Get<float>(emptyString), 0.0, 1e-6);
  elem->AddValue("float", "12.34", "0", "description");
  EXPECT_NEAR(elem->Get<float>(emptyString), 12.34, 1e-6);

  elem.reset(new sdf::Element());
  EXPECT_NEAR(elem->Get<double>(emptyString), 0.0, 1e-6);
  elem->AddValue("double", "12.34", "0", "description");
  EXPECT_NEAR(elem->Get<double>(emptyString), 12.34, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(RmlUpdate, GetAny)
{
  std::ostringstream stream;
  // Test types double, bool, string, int, vector3, color, pose
  stream << "<sdf version='1.5'>"
         << "<world name='test'>"
         << "   <physics type='ode'>"
         << "     <gravity> 0 0 -7.1 </gravity>"
         << "     <max_contacts>8</max_contacts>"
         << "     <max_step_size>0.002</max_step_size>"
         << "   </physics>"
         << "   <model name='test_model'>"
         << "     <pose>0 1 2 0 0 0</pose>"
         << "     <static>true</static>"
         << "     <link name='link1'>"
         << "       <visual name='visual'>"
         << "         <material>"
         << "           <ambient>0.1 0.1 0.1 1</ambient>"
         << "         </material>"
         << "       </visual>"
         << "     </link>"
         << "   </model>"
         << "</world>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed.root->GetElement("world");

  EXPECT_TRUE(worldElem->HasElement("model"));
  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  EXPECT_TRUE(worldElem->HasElement("physics"));
  sdf::ElementPtr physicsElem = worldElem->GetElement("physics");

  {
    boost::any anyValue = modelElem->GetAny("name");
    try
    {
      EXPECT_EQ(boost::any_cast<std::string>(anyValue), "test_model");
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(modelElem->HasElement("pose"));
    sdf::ElementPtr poseElem = modelElem->GetElement("pose");
    boost::any anyValue = poseElem->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<sdf::Pose>(anyValue),
          sdf::Pose(0, 1, 2, 0, 0, 0));
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(physicsElem->HasElement("gravity"));
    boost::any anyValue = physicsElem->GetElement("gravity")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<sdf::Vector3>(anyValue),
          sdf::Vector3(0, 0, -7.1));
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(physicsElem->HasElement("max_step_size"));
    boost::any anyValue = physicsElem->GetElement("max_step_size")->GetAny();
    try
    {
      EXPECT_NEAR(boost::any_cast<double>(anyValue), 0.002, 1e-6);
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(physicsElem->HasElement("max_contacts"));
    boost::any anyValue = physicsElem->GetElement("max_contacts")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<int>(anyValue), 8);
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(physicsElem->HasElement("gravity"));
    boost::any anyValue = physicsElem->GetElement("gravity")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<sdf::Vector3>(anyValue),
          sdf::Vector3(0, 0, -7.1));
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(modelElem->HasElement("static"));
    boost::any anyValue = modelElem->GetElement("static")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<bool>(anyValue), true);
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(modelElem->HasElement("link"));
    EXPECT_TRUE(modelElem->GetElement("link")->HasElement("visual"));
    EXPECT_TRUE(modelElem->GetElement("link")->GetElement("visual")->
        HasElement("material"));
    sdf::ElementPtr materialElem = modelElem->GetElement("link")->
        GetElement("visual")->GetElement("material");
    EXPECT_TRUE(materialElem->HasElement("ambient"));
    boost::any anyValue = materialElem->GetElement("ambient")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<sdf::Color>(anyValue),
          sdf::Color(0.1, 0.1, 0.1, 1));
    }
    catch(boost::bad_any_cast &_e)
    {
      FAIL();
    }
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
