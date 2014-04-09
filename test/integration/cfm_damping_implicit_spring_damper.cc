/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include <map>
#include "sdf/sdf.hh"

#include "test_config.h"

const std::string URDF_TEST_FILE = std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/cfm_damping_implicit_spring_damper.urdf";

const std::string SDF_TEST_FILE = std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/cfm_damping_implicit_spring_damper.sdf";

/////////////////////////////////////////////////
TEST(SDFParser, CFMDampingSDFTest)
{
  char *pathCStr = getenv("SDF_PATH");
  boost::filesystem::path path = PROJECT_SOURCE_PATH;
  path = path / "sdf" / SDF_VERSION;
  setenv("SDF_PATH", path.string().c_str(), 1);

  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(SDF_TEST_FILE, robot));
  if (pathCStr)
  {
    setenv("SDF_PATH", pathCStr, 1);
  }
  else
  {
    unsetenv("SDF_PATH");
  }

  sdf::ElementPtr model = robot->root->GetElement("model");
  for (sdf::ElementPtr joint = model->GetElement("joint"); joint;
       joint = joint->GetNextElement("joint"))
  {
    std::string jointName = joint->Get<std::string>("name");
    if (jointName == "jointw1")
    {
      // No cfm_damping tag was specified
      EXPECT_FALSE(joint->HasElement("physics"));
    }
    else if (jointName == "joint12a")
    {
      // cfm_damping = 0
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_FALSE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
    }
    else if (jointName == "joint12b")
    {
      // cfm_damping = true
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_FALSE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
    }
    else if (jointName == "joint12c")
    {
      // implicit_spring_damper = 0
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_TRUE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
      EXPECT_FALSE(physics->GetElement("ode")->Get<bool>(
            "implicit_spring_damper"));
    }
    else if (jointName == "joint12d")
    {
      // implicit_spring_damper = true
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_TRUE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
      EXPECT_TRUE(physics->GetElement("ode")->Get<bool>(
            "implicit_spring_damper"));
    }
  }
}

/////////////////////////////////////////////////
TEST(SDFParser, CFMDampingURDFTest)
{
  char *pathCStr = getenv("SDF_PATH");
  boost::filesystem::path path = PROJECT_SOURCE_PATH;
  path = path / "sdf" / SDF_VERSION;
  setenv("SDF_PATH", path.string().c_str(), 1);

  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(URDF_TEST_FILE, robot));
  if (pathCStr)
  {
    setenv("SDF_PATH", pathCStr, 1);
  }
  else
  {
    unsetenv("SDF_PATH");
  }

  sdf::ElementPtr model = robot->root->GetElement("model");
  for (sdf::ElementPtr joint = model->GetElement("joint"); joint;
       joint = joint->GetNextElement("joint"))
  {
    std::string jointName = joint->Get<std::string>("name");
    if (jointName == "jointw0")
    {
      // No cfm_damping tag was specified
      EXPECT_FALSE(joint->HasElement("physics"));
    }
    else if (jointName == "joint01")
    {
      // cfmDamping = true
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("implicit_spring_damper"));
      EXPECT_TRUE(physics->Get<bool>("implicit_spring_damper"));
    }
    else if (jointName == "joint12")
    {
      // cfmDamping not provided
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      EXPECT_FALSE(physics->HasElement("implicit_spring_damper"));
    }
    else if (jointName == "joint13")
    {
      // implicitSpringDamper = 1
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("implicit_spring_damper"));
      EXPECT_TRUE(physics->Get<bool>("implicit_spring_damper"));
    }
  }
}
