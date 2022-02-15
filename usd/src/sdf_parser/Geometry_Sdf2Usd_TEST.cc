/*
 * Copyright 2022 Open Source Robotics Foundation
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

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Util.hh>

#include "sdf/usd/sdf_parser/World.hh"
#include "sdf/Root.hh"
#include "test_config.h"
#include "test_utils.hh"
#include "../UsdTestUtils.hh"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
class UsdStageFixture : public::testing::Test
{
  public: UsdStageFixture() = default;

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
    ASSERT_TRUE(this->stage);
  }

  public: pxr::UsdStageRefPtr stage;
};

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Geometry)
{
  sdf::setFindCallback(sdf::usd::testing::findFileCb);
  ignition::common::addFindFileURICallback(
    std::bind(&sdf::usd::testing::FindResourceUri, std::placeholders::_1));

  const auto path = sdf::testing::TestFile("sdf", "basic_shapes.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);

  std::string groundPlanePath = worldPath + "/" + "ground_plane";
  std::string groundPlaneLinkPath = groundPlanePath + "/" + "link";
  std::string groundPlaneVisualPath = groundPlaneLinkPath + "/" + "visual";
  std::string groundPlaneGeometryPath =
    groundPlaneVisualPath + "/" + "geometry";

  pxr::GfVec3f scale;
  double size, height, radius, length;

  auto groundPlaneGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(groundPlaneGeometryPath));
  ASSERT_TRUE(groundPlaneGeometry);
  auto usdGroundPlane = pxr::UsdGeomCube(groundPlaneGeometry);
  ASSERT_TRUE(usdGroundPlane);
  usdGroundPlane.GetSizeAttr().Get(&size);
  EXPECT_DOUBLE_EQ(1.0, size);
  auto scaleAttr =
    groundPlaneGeometry.GetAttribute(pxr::TfToken("xformOp:scale"));
  scaleAttr.Get(&scale);
  EXPECT_EQ(pxr::GfVec3f(2, 4, 0.25), scale);

  std::string boxPath = worldPath + "/" + "box";
  std::string boxLinkPath = boxPath + "/" + "link";
  std::string boxVisualPath = boxLinkPath + "/" + "box_vis";
  std::string boxGeometryPath =
    boxVisualPath + "/" + "geometry";
  auto boxGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(boxGeometryPath));
  ASSERT_TRUE(boxGeometry);
  auto usdCube = pxr::UsdGeomCube(boxGeometry);
  ASSERT_TRUE(usdCube);
  usdCube.GetSizeAttr().Get(&size);
  EXPECT_DOUBLE_EQ(1.0, size);
  scaleAttr = boxGeometry.GetAttribute(pxr::TfToken("xformOp:scale"));
  scaleAttr.Get(&scale);
  EXPECT_EQ(pxr::GfVec3f(1, 2, 3), scale);

  std::string cylinderPath = worldPath + "/" + "cylinder";
  std::string cylinderLinkPath = cylinderPath + "/" + "link";
  std::string cylinderVisualPath = cylinderLinkPath + "/" + "visual";
  std::string cylinderGeometryPath =
    cylinderVisualPath + "/" + "geometry";
  auto cylinderGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(cylinderGeometryPath));
  ASSERT_TRUE(cylinderGeometry);
  auto usdCylinder = pxr::UsdGeomCylinder(cylinderGeometry);
  ASSERT_TRUE(usdCylinder);
  usdCylinder.GetRadiusAttr().Get(&radius);
  usdCylinder.GetHeightAttr().Get(&height);
  EXPECT_DOUBLE_EQ(1.0, size);
  EXPECT_DOUBLE_EQ(0.2, radius);
  EXPECT_DOUBLE_EQ(0.1, height);

  std::string spherePath = worldPath + "/" + "sphere";
  std::string sphereLinkPath = spherePath + "/" + "link";
  std::string sphereVisualPath = sphereLinkPath + "/" + "sphere_vis";
  std::string sphereGeometryPath =
    sphereVisualPath + "/" + "geometry";
  auto sphereGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(sphereGeometryPath));
  ASSERT_TRUE(sphereGeometry);
  auto usdsphere = pxr::UsdGeomSphere(sphereGeometry);
  ASSERT_TRUE(usdsphere);
  usdsphere.GetRadiusAttr().Get(&radius);
  EXPECT_DOUBLE_EQ(1.0, size);
  EXPECT_DOUBLE_EQ(1.23, radius);

  std::string capsulePath = worldPath + "/" + "capsule";
  std::string capsuleLinkPath = capsulePath + "/" + "link";
  std::string capsuleVisualPath = capsuleLinkPath + "/" + "visual";
  std::string capsuleGeometryPath =
    capsuleVisualPath + "/" + "geometry";
  auto capsuleGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(capsuleGeometryPath));
  ASSERT_TRUE(capsuleGeometry);
  auto usdcapsule = pxr::UsdGeomCapsule(capsuleGeometry);
  ASSERT_TRUE(usdcapsule);
  usdcapsule.GetRadiusAttr().Get(&radius);
  usdcapsule.GetHeightAttr().Get(&length);
  EXPECT_DOUBLE_EQ(1.0, size);
  EXPECT_DOUBLE_EQ(0.2, radius);
  EXPECT_DOUBLE_EQ(0.1, length);

  std::string meshPath = worldPath + "/" + "mesh";
  std::string meshLinkPath = meshPath + "/" + "link";
  std::string meshVisualPath = meshLinkPath + "/" + "visual";
  std::string meshGeometryPath =
    meshVisualPath + "/" + "geometry";
  std::string meshGeometryMeshPath =
    meshGeometryPath + "/" + "Cube";
  auto meshGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(meshGeometryMeshPath));
  ASSERT_TRUE(meshGeometry);
  auto usdmesh = pxr::UsdGeomMesh(meshGeometry);
  ASSERT_TRUE(usdmesh);

  pxr::VtArray<pxr::GfVec3f> normals;
  pxr::VtArray<pxr::GfVec3f> points;
  pxr::VtIntArray faceVertexIndices;
  pxr::VtIntArray faceVertexCounts;
  meshGeometry.GetAttribute(pxr::TfToken("faceVertexCounts")).
    Get(&faceVertexCounts);
  meshGeometry.GetAttribute(pxr::TfToken("faceVertexIndices")).
    Get(&faceVertexIndices);
  meshGeometry.GetAttribute(pxr::TfToken("normals")).Get(&normals);
  meshGeometry.GetAttribute(pxr::TfToken("points")).Get(&points);
  EXPECT_EQ(24u, normals.size());
  EXPECT_EQ(24u, points.size());
  EXPECT_EQ(36u, faceVertexIndices.size());
  EXPECT_EQ(12u, faceVertexCounts.size());
}
