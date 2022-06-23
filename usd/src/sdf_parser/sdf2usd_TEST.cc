/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/common/Filesystem.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.h"
#include "test_utils.hh"

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#ifdef _WIN32
  #define popen  _popen
  #define pclose _pclose
#endif

static std::string sdf2usdCommand()
{
  return gz::common::joinPaths(std::string(PROJECT_BINARY_DIR), "bin",
      "sdf2usd");
}

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(check_cmd, GZ_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase = gz::common::joinPaths(pathBase, "test", "sdf");

  auto tmpDir = gz::common::tempDirectoryPath();
  auto tmp = gz::common::createTempDirectory("usd", tmpDir);
  // Check a good SDF file
  {
    std::string path = gz::common::joinPaths(pathBase,
      "lights.sdf");
    const auto outputUsdFilePath =
      gz::common::joinPaths(tmp, "shapes.usd");
    EXPECT_FALSE(gz::common::isFile(outputUsdFilePath));
    std::string output =
      custom_exec_str(sdf2usdCommand() + " " + path + " " + outputUsdFilePath);
    // TODO(adlarkin) make sure 'output' (i.e., the result of running the
    // sdf2usd executable) is an empty string once the usd2sdf parser is fully
    // implemented (right now, running the parser outputs an error indicating
    // that functionality isn't complete)

    // make sure that a shapes.usd file was generated
    EXPECT_TRUE(gz::common::isFile(outputUsdFilePath)) << output;

    // TODO(ahcorde): Check the contents of outputUsdFilePath when the parser
    // is implemented
  }
}

// helper method for getting attributes of a prim
template<typename T>
void getAttr(const pxr::UsdPrim &_prim, const std::string &_attributeName,
    T &_value)
{
  const auto attr = _prim.GetAttribute(pxr::TfToken(_attributeName));
  ASSERT_TRUE(attr);
  attr.Get(&_value);
}

TEST(check_cmd_model, GZ_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase = gz::common::joinPaths(pathBase, "test", "sdf");

  auto tmpDir = gz::common::tempDirectoryPath();
  auto tmp = gz::common::createTempDirectory("usd", tmpDir);
  // Check a good SDF file
  {
    std::string path = gz::common::joinPaths(pathBase,
      "ellipsoid_model.sdf");
    const auto outputUsdFilePath =
      gz::common::joinPaths(tmp, "ellipsoid.usd");
    EXPECT_FALSE(gz::common::isFile(outputUsdFilePath));
    std::string output =
      custom_exec_str(sdf2usdCommand() + " " + path + " " + outputUsdFilePath);
    EXPECT_TRUE(output.empty());

    // make sure that a ellipsoid.usd file was generated
    EXPECT_TRUE(gz::common::isFile(outputUsdFilePath)) << output;

    const auto stage = pxr::UsdStage::Open(outputUsdFilePath);
    ASSERT_TRUE(stage);

    ASSERT_TRUE(stage->GetPrimAtPath(pxr::SdfPath("/ellipsoid")));
    ASSERT_TRUE(stage->GetPrimAtPath(
      pxr::SdfPath("/ellipsoid/ellipsoid_link")));
    ASSERT_TRUE(stage->GetPrimAtPath(
      pxr::SdfPath("/ellipsoid/ellipsoid_link/ellipsoid_visual")));
    ASSERT_TRUE(stage->GetPrimAtPath(
      pxr::SdfPath("/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry")));

    // check material physics
    const auto materialPhysicsPrim =
      stage->GetPrimAtPath(pxr::SdfPath("/Looks/MaterialPhysics_0"));
    ASSERT_TRUE(materialPhysicsPrim);
    float floatAttrVal;
    getAttr(materialPhysicsPrim, "physics:density", floatAttrVal);
    EXPECT_FLOAT_EQ(floatAttrVal, 1.0f);
    getAttr(materialPhysicsPrim, "physics:dynamicFriction", floatAttrVal);
    EXPECT_FLOAT_EQ(floatAttrVal, 0.6f);
    getAttr(materialPhysicsPrim, "physics:restitution", floatAttrVal);
    EXPECT_FLOAT_EQ(floatAttrVal, 1.0f);
    getAttr(materialPhysicsPrim, "physics:staticFriction", floatAttrVal);
    EXPECT_FLOAT_EQ(floatAttrVal, 0.7f);
    pxr::TfToken tokenAttrVal;
    getAttr(materialPhysicsPrim, "physXMaterial:frictionCombineMode",
        tokenAttrVal);
    EXPECT_EQ(tokenAttrVal, pxr::TfToken("average"));
    getAttr(materialPhysicsPrim, "physXMaterial:restitutionCombineMode",
        tokenAttrVal);
    EXPECT_EQ(tokenAttrVal, pxr::TfToken("average"));

    // make sure the collision's friction parameters were properly converted
    const auto collisionGeometryPrim = stage->GetPrimAtPath(
      pxr::SdfPath("/ellipsoid/ellipsoid_link/ellipsoid_collision/geometry"));
    ASSERT_TRUE(collisionGeometryPrim);
    const pxr::TfToken frictionRelKey("material:binding");
    ASSERT_TRUE(collisionGeometryPrim.HasRelationship(frictionRelKey));
    const auto rel = collisionGeometryPrim.GetRelationship(frictionRelKey);
    pxr::SdfPathVector paths;
    rel.GetTargets(&paths);
    ASSERT_EQ(1u, paths.size());
    EXPECT_EQ(pxr::SdfPath("/Looks/MaterialPhysics_0"), paths[0]);
  }
}
