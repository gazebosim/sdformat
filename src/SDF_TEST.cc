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
#include <boost/any.hpp>
#include <ignition/math.hh>

#include "sdf/sdf.hh"

class SDFUpdateFixture
{
  public:  std::string GetName() const {return this->name;}
  public:  bool GetFlag() const {return this->flag;}
  public:  ignition::math::Pose3d GetPose() const {return this->pose;}
  public:  std::string name;
  public:  bool flag;
  public:  ignition::math::Pose3d pose;
};

////////////////////////////////////////////////////
/// Ensure that SDF::Update is working for attributes
TEST(SDF, UpdateAttribute)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "  <model name='test_model'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  ASSERT_TRUE(sdfParsed.Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.Root()->GetElement("model");

  // Read name attribute value
  ASSERT_TRUE(modelElem->HasAttribute("name"));
  sdf::ParamPtr nameParam = modelElem->GetAttribute("name");
  EXPECT_TRUE(nameParam->IsType<std::string>());

  // Set test class variables based on sdf values
  // Set parameter update functions to test class accessors
  SDFUpdateFixture fixture;
  nameParam->Get(fixture.name);
  nameParam->SetUpdateFunc(std::bind(&SDFUpdateFixture::GetName, &fixture));

  std::string nameCheck;
  int i;
  for (i = 0; i < 4; i++)
  {
    // Update test class variables
    fixture.name[0] = 'd' + static_cast<char>(i);

    // Update root sdf element
    sdfParsed.Root()->Update();

    // Expect sdf values to match test class variables
    nameParam->Get(nameCheck);
    EXPECT_STREQ(nameCheck.c_str(), fixture.name.c_str());
  }
}

////////////////////////////////////////////////////
/// Ensure that SDF::Update is working for elements
TEST(SDF, UpdateElement)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "  <model name='test_model'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  ASSERT_TRUE(sdfParsed.Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.Root()->GetElement("model");

  // Read static element value
  ASSERT_TRUE(modelElem->HasElement("static"));
  sdf::ParamPtr staticParam = modelElem->GetElement("static")->GetValue();
  EXPECT_TRUE(staticParam->IsType<bool>());

  // Read pose element value
  ASSERT_TRUE(modelElem->HasElement("pose"));
  sdf::ParamPtr poseParam = modelElem->GetElement("pose")->GetValue();
  EXPECT_TRUE(poseParam->IsType<ignition::math::Pose3d>());

  // Set test class variables based on sdf values
  // Set parameter update functions to test class accessors
  SDFUpdateFixture fixture;
  staticParam->Get(fixture.flag);
  staticParam->SetUpdateFunc(std::bind(&SDFUpdateFixture::GetFlag, &fixture));
  poseParam->Get(fixture.pose);
  poseParam->SetUpdateFunc(std::bind(&SDFUpdateFixture::GetPose, &fixture));

  bool flagCheck;
  ignition::math::Pose3d poseCheck;
  int i;
  for (i = 0; i < 4; i++)
  {
    // Update test class variables
    fixture.flag = !fixture.flag;
    fixture.pose.Pos().X() = i;
    fixture.pose.Pos().Y() = i+10;
    fixture.pose.Pos().Z() = -i*i*i;

    // Update root sdf element
    sdfParsed.Root()->Update();

    // Expect sdf values to match test class variables
    staticParam->Get(flagCheck);
    EXPECT_EQ(flagCheck, fixture.flag);
    poseParam->Get(poseCheck);
    EXPECT_EQ(poseCheck, fixture.pose);
  }
}

////////////////////////////////////////////////////
/// Ensure that SDF::Element::RemoveFromParent is working
TEST(SDF, ElementRemoveFromParent)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "  <model name='model1'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "  <model name='model2'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "  <model name='model3'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr elem;

  // Verify correct parsing
  ASSERT_TRUE(sdfParsed.Root()->HasElement("model"));
  elem = sdfParsed.Root()->GetElement("model");

  // Select the second model named 'model2'
  elem = elem->GetNextElement("model");
  ASSERT_TRUE(elem != nullptr);
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model2");
  EXPECT_EQ(elem->Get<std::string>("name", "default_value").first, "model2");
  EXPECT_TRUE(elem->Get<std::string>("name", "default_value").second);

  // Remove model2
  elem->RemoveFromParent();

  // Get first model element again
  elem = sdfParsed.Root()->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model1");
  EXPECT_EQ(elem->Get<std::string>("bad_name", "default").first, "default");
  EXPECT_FALSE(elem->Get<std::string>("bad_name", "default").second);

  std::string value;
  bool success;
  std::tie(value, success) = elem->Get<std::string>("bad_name", "default");
  EXPECT_FALSE(success);
  EXPECT_EQ(value, "default");

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
TEST(SDF, ElementRemoveChild)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "  <model name='model1'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "  <model name='model2'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "  <model name='model3'>"
         << "    <pose>0 1 2  0 0 0</pose>"
         << "    <static>false</static>"
         << "  </model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr elem, elem2;

  // Verify correct parsing
  ASSERT_TRUE(sdfParsed.Root()->HasElement("model"));
  elem = sdfParsed.Root()->GetElement("model");

  // Select the static element in model1
  elem2 = elem->GetElement("static");
  EXPECT_TRUE(elem2 != nullptr);
  EXPECT_FALSE(elem2->Get<bool>());
  elem->RemoveChild(elem2);

  // Get first model element again
  elem = sdfParsed.Root()->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->Get<std::string>("name"), "model1");

  // Check that we have deleted the static element in model1
  EXPECT_FALSE(elem->HasElement("static"));

  // Get model2
  elem2 = elem->GetNextElement("model");

  // Remove model2
  sdfParsed.Root()->RemoveChild(elem2);

  // Get first model element again
  elem = sdfParsed.Root()->GetElement("model");
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
TEST(SDF, EmptyValues)
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
  EXPECT_EQ(elem->Get<ignition::math::Vector2d>(emptyString),
      ignition::math::Vector2d());
  elem->AddValue("vector2d", "1 2", "0", "description");
  EXPECT_EQ(elem->Get<ignition::math::Vector2d>(emptyString),
      ignition::math::Vector2d(1, 2));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<ignition::math::Vector3d>(emptyString),
      ignition::math::Vector3d());
  elem->AddValue("vector3", "1 2 3", "0", "description");
  EXPECT_EQ(elem->Get<ignition::math::Vector3d>(emptyString),
      ignition::math::Vector3d(1, 2, 3));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<ignition::math::Quaterniond>(emptyString),
            ignition::math::Quaterniond());
  elem->AddValue("quaternion", "1 2 3", "0", "description");
  EXPECT_EQ(elem->Get<ignition::math::Quaterniond>(emptyString),
            ignition::math::Quaterniond(-2.14159, 1.14159, -0.141593));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<ignition::math::Pose3d>(emptyString),
      ignition::math::Pose3d());
  elem->AddValue("pose", "1.0 2.0 3.0 4.0 5.0 6.0", "0", "description");
  EXPECT_EQ(elem->Get<ignition::math::Pose3d>(emptyString).Pos(),
      ignition::math::Pose3d(1, 2, 3, 4, 5, 6).Pos());
  EXPECT_EQ(elem->Get<ignition::math::Pose3d>(emptyString).Rot().Euler(),
      ignition::math::Pose3d(1, 2, 3, 4, 5, 6).Rot().Euler());

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->Get<ignition::math::Color>(emptyString),
      ignition::math::Color());
  elem->AddValue("color", ".1 .2 .3 1.0", "0", "description");
  EXPECT_EQ(elem->Get<ignition::math::Color>(emptyString),
            ignition::math::Color(.1f, .2f, .3f, 1.0f));

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
TEST(SDF, GetAny)
{
  std::ostringstream stream;
  // Test types double, bool, string, int, vector3, color, pose
  stream << "<sdf version='1.6'>"
         << "  <world name='test'>"
         << "     <gravity> 0 0 -7.1 </gravity>"
         << "     <physics type='ode'>"
         << "       <max_contacts>8</max_contacts>"
         << "       <max_step_size>0.002</max_step_size>"
         << "     </physics>"
         << "     <model name='test_model'>"
         << "       <pose>0 1 2 0 0 0</pose>"
         << "       <static>true</static>"
         << "       <link name='link1'>"
         << "         <visual name='visual'>"
         << "           <material>"
         << "             <ambient>0.1 0.1 0.1 1</ambient>"
         << "           </material>"
         << "         </visual>"
         << "       </link>"
         << "     </model>"
         << "  </world>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  ASSERT_TRUE(sdfParsed.Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed.Root()->GetElement("world");

  ASSERT_TRUE(worldElem->HasElement("model"));
  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  ASSERT_TRUE(worldElem->HasElement("physics"));
  sdf::ElementPtr physicsElem = worldElem->GetElement("physics");

  {
    boost::any anyValue = modelElem->GetAny("name");
    try
    {
      EXPECT_EQ(boost::any_cast<std::string>(anyValue), "test_model");
    }
    catch(boost::bad_any_cast &/*_e*/)
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
      EXPECT_EQ(boost::any_cast<ignition::math::Pose3d>(anyValue),
          ignition::math::Pose3d(0, 1, 2, 0, 0, 0));
    }
    catch(boost::bad_any_cast &/*_e*/)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(worldElem->HasElement("gravity"));
    boost::any anyValue = worldElem->GetElement("gravity")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<ignition::math::Vector3d>(anyValue),
          ignition::math::Vector3d(0, 0, -7.1));
    }
    catch(boost::bad_any_cast &/*_e*/)
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
    catch(boost::bad_any_cast &/*_e*/)
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
    catch(boost::bad_any_cast &/*_e*/)
    {
      FAIL();
    }
  }

  {
    EXPECT_TRUE(worldElem->HasElement("gravity"));
    boost::any anyValue = worldElem->GetElement("gravity")->GetAny();
    try
    {
      EXPECT_EQ(boost::any_cast<ignition::math::Vector3d>(anyValue),
          ignition::math::Vector3d(0, 0, -7.1));
    }
    catch(boost::bad_any_cast &/*_e*/)
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
    catch(boost::bad_any_cast &/*_e*/)
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
      EXPECT_EQ(boost::any_cast<ignition::math::Color>(anyValue),
          ignition::math::Color(0.1f, 0.1f, 0.1f, 1.0f));
    }
    catch(boost::bad_any_cast &/*_e*/)
    {
      FAIL();
    }
  }
}

/////////////////////////////////////////////////
TEST(SDF, Version)
{
  EXPECT_STREQ(SDF_VERSION, sdf::SDF::Version().c_str());

  sdf::SDF::Version("0.2.3");
  EXPECT_STREQ("0.2.3", sdf::SDF::Version().c_str());
}

/////////////////////////////////////////////////
TEST(SDF, PrintDoc)
{
  std::stringstream buffer;
  auto old = std::cout.rdbuf(buffer.rdbuf());

  sdf::SDF sdf;
  sdf.PrintDoc();

  EXPECT_GT(buffer.str().size(), 2000u);
  EXPECT_NE(buffer.str().find("HTML"), std::string::npos);
  EXPECT_NE(buffer.str().find("SDF"), std::string::npos);
  EXPECT_NE(buffer.str().find("Usage"), std::string::npos);
  EXPECT_NE(buffer.str().find("Meta-Tags"), std::string::npos);

  std::cout.rdbuf(old);
}

/////////////////////////////////////////////////
TEST(SDF, PrintDescription)
{
  std::stringstream buffer;
  auto old = std::cout.rdbuf(buffer.rdbuf());

  sdf::SDF sdf;
  sdf.PrintDescription();

  EXPECT_GT(buffer.str().size(), 70u);
  EXPECT_NE(buffer.str().find("<element"), std::string::npos);
  EXPECT_NE(buffer.str().find("<description>"), std::string::npos);

  std::cout.rdbuf(old);
}

/////////////////////////////////////////////////
TEST(SDF, PrintValues)
{
  std::stringstream buffer;
  auto old = std::cout.rdbuf(buffer.rdbuf());

  sdf::SDF sdf;
  sdf.PrintValues();

  EXPECT_GT(buffer.str().size(), 3u);
  EXPECT_NE(buffer.str().find("</>"), std::string::npos);

  std::cout.rdbuf(old);
}

#ifndef _WIN32
bool create_new_temp_dir(std::string &_new_temp_path)
{
  std::string tmppath;
  const char *tmp = getenv("TMPDIR");
  if (tmp)
  {
    tmppath = std::string(tmp);
  }
  else
  {
    tmppath = std::string("/tmp");
  }

  tmppath += "/XXXXXX";

  char *dtemp = mkdtemp(const_cast<char *>(tmppath.c_str()));
  if (dtemp == nullptr)
  {
    return false;
  }

  _new_temp_path = std::string(dtemp);

  return true;
}

/////////////////////////////////////////////////
bool g_findFileCbCalled = false;
std::string findFileCb(const std::string &)
{
  g_findFileCbCalled = true;
  return "coconut";
}

/////////////////////////////////////////////////
TEST(SDF, WriteURIPath)
{
  // Create temp dir
  std::string tempDir;
  ASSERT_TRUE(create_new_temp_dir(tempDir));

  // Write to file
  auto tempFile = tempDir + "/test.sdf";

  sdf::SDF sdf;
  sdf.Write(tempFile);

  // Check file was created
  auto fp = fopen(tempFile.c_str(), "r");
  ASSERT_NE(nullptr, fp);

  // Add temp dir to path
  sdf::addURIPath("test://", tempDir);

  // Find file
  EXPECT_EQ(sdf::findFile("test://test.sdf"), tempFile);

  // Can't find file, fallback to user callback
  EXPECT_EQ(sdf::findFile("banana", false, true), "");

  sdf::setFindCallback(findFileCb);
  EXPECT_EQ(sdf::findFile("banana", false, true), "coconut");

  // Check callback was called
  EXPECT_TRUE(g_findFileCbCalled);

  // Cleanup
  ASSERT_EQ(std::remove(tempFile.c_str()), 0);
  ASSERT_EQ(rmdir(tempDir.c_str()), 0);
}
#endif  // _WIN32

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
