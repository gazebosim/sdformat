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

#include <gtest/gtest.h>

#include <optional>
#include <string>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "test_config.h"
#include "test_utils.hh"

#include "USDLinks.hh"

#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/Visual.hh"

/////////////////////////////////////////////////
TEST(USDLinksTest, LinksNameMassAndDiagonalMoments)
{
  const std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  const auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  sdf::usd::USDData usdData(filename);
  usdData.Init();

  ignition::math::Vector3d scale(1, 1, 1);

  auto checkLink =
    [](const sdf::Link &_link,
       const std::string &_name,
       double mass,
       const ignition::math::Vector3d &_diagonalMoment,
       unsigned int _nvisual,
       sdf::GeometryType _type)
    {
      EXPECT_EQ(_name, _link.Name());
      EXPECT_NEAR(mass, _link.Inertial().MassMatrix().Mass(), 1e-4);
      EXPECT_EQ(
        _diagonalMoment,
        _link.Inertial().MassMatrix().PrincipalMoments());
      EXPECT_EQ(_nvisual, _link.VisualCount());
      if (_nvisual)
      {
        auto visual = _link.VisualByIndex(0);
        ASSERT_NE(nullptr, visual);
        auto geom = visual->Geom();
        ASSERT_NE(nullptr, geom);
        EXPECT_EQ(_type, geom->Type());
        auto collision = _link.CollisionByIndex(0);
        ASSERT_NE(nullptr, collision);
        auto geomCol = collision->Geom();
        ASSERT_NE(nullptr, geomCol);
      }
    };

  const auto boxLink = stage->GetPrimAtPath(pxr::SdfPath("/box/box_link"));
  ASSERT_TRUE(boxLink);

  std::optional<sdf::Link> linkSDF;
  sdf::usd::ParseUSDLinks(
    boxLink, "/box/box_link", linkSDF, usdData, scale);

  EXPECT_EQ(ignition::math::Vector3d(1, 0.1, 1), scale);

  const auto boxLinkGeometry = stage->GetPrimAtPath(
    pxr::SdfPath("/box/box_link/box_visual/geometry"));
  ASSERT_TRUE(boxLinkGeometry);

  sdf::usd::ParseUSDLinks(
    boxLinkGeometry, "/box/box_link", linkSDF, usdData, scale);

  const auto boxLinkCollision = stage->GetPrimAtPath(
    pxr::SdfPath("/box/box_link/box_visual/collision"));
  ASSERT_TRUE(boxLinkCollision);

  sdf::usd::ParseUSDLinks(
    boxLinkCollision, "/box/box_link", linkSDF, usdData, scale);

  ASSERT_TRUE(linkSDF);
  checkLink(linkSDF.value(), "box_link", 1.0,
            ignition::math::Vector3d(0.1666, 0.1666, 0.1666),
            1u, sdf::GeometryType::BOX);

  const auto cylinderLink = stage->GetPrimAtPath(
    pxr::SdfPath("/cylinder/cylinder_link"));
  ASSERT_TRUE(cylinderLink);

  std::optional<sdf::Link> linkCylinderSDF;
  sdf::usd::ParseUSDLinks(
    cylinderLink, "/cylinder/cylinder_link",
    linkCylinderSDF, usdData, scale);

  const auto cylinderLinkGeometry = stage->GetPrimAtPath(
    pxr::SdfPath("/cylinder/cylinder_link/cylinder_visual/geometry"));
  EXPECT_TRUE(cylinderLinkGeometry);

  sdf::usd::ParseUSDLinks(
    cylinderLinkGeometry, "/cylinder/cylinder_link",
    linkCylinderSDF, usdData, scale);

  const auto cylinderLinkCollision = stage->GetPrimAtPath(
    pxr::SdfPath("/cylinder/cylinder_link/cylinder_visual/collision"));
  EXPECT_TRUE(cylinderLinkCollision);

  sdf::usd::ParseUSDLinks(
    cylinderLinkCollision, "/cylinder/cylinder_link",
    linkCylinderSDF, usdData, scale);

  checkLink(linkCylinderSDF.value(), "cylinder_link", 1.7,
            ignition::math::Vector3d(0.1458, 0.1458, 0.125),
            1u, sdf::GeometryType::CYLINDER);

  const auto sphereLink = stage->GetPrimAtPath(
    pxr::SdfPath("/sphere/sphere_link"));
  ASSERT_TRUE(sphereLink);

  std::optional<sdf::Link> linkSphereSDF;
  sdf::usd::ParseUSDLinks(
    sphereLink, "/sphere/sphere_link", linkSphereSDF, usdData, scale);

  const auto sphereLinkGeometry = stage->GetPrimAtPath(
    pxr::SdfPath("/sphere/sphere_link/sphere_visual/geometry"));
  EXPECT_TRUE(sphereLinkGeometry);

  sdf::usd::ParseUSDLinks(
    sphereLinkGeometry, "/sphere/sphere_link",
    linkSphereSDF, usdData, scale);

  const auto sphereLinkCollision = stage->GetPrimAtPath(
    pxr::SdfPath("/sphere/sphere_link/sphere_visual/collision"));
  EXPECT_TRUE(sphereLinkCollision);

  sdf::usd::ParseUSDLinks(
    sphereLinkCollision, "/sphere/sphere_link",
    linkSphereSDF, usdData, scale);

  ASSERT_TRUE(linkSphereSDF);
  checkLink(linkSphereSDF.value(), "sphere_link", 2,
            ignition::math::Vector3d(0.1, 0.1, 0.1),
            1u, sdf::GeometryType::SPHERE);

  const auto capsuleLink = stage->GetPrimAtPath(
    pxr::SdfPath("/capsule/capsule_link"));
  ASSERT_TRUE(capsuleLink);

  std::optional<sdf::Link> linkCapsuleSDF;
  sdf::usd::ParseUSDLinks(
    capsuleLink, "/capsule/capsule_link", linkCapsuleSDF, usdData, scale);

  const auto capsuleLinkCollision = stage->GetPrimAtPath(
    pxr::SdfPath("/capsule/capsule_link/capsule_visual/collision"));
  EXPECT_TRUE(capsuleLinkCollision);

  sdf::usd::ParseUSDLinks(
    capsuleLinkCollision, "/capsule/capsule_link",
    linkCapsuleSDF, usdData, scale);

  checkLink(linkCapsuleSDF.value(), "capsule_link", 1,
            ignition::math::Vector3d(0.074154, 0.074154, 0.018769),
            0u, sdf::GeometryType::EMPTY);

  const auto ellipsoidLink = stage->GetPrimAtPath(
    pxr::SdfPath("/ellipsoid/ellipsoid_link"));
  ASSERT_TRUE(ellipsoidLink);

  std::optional<sdf::Link> linkEllipsoidSDF;
  sdf::usd::ParseUSDLinks(
    ellipsoidLink, "/ellipsoid/ellipsoid_link",
    linkEllipsoidSDF, usdData, scale);

  const auto ellipsoidLinkCollision = stage->GetPrimAtPath(
    pxr::SdfPath("/ellipsoid/ellipsoid_link/ellipsoid_visual/collision"));
  EXPECT_TRUE(ellipsoidLinkCollision);

  sdf::usd::ParseUSDLinks(
    ellipsoidLinkCollision, "/ellipsoid/ellipsoid_link",
    linkEllipsoidSDF, usdData, scale);

  checkLink(linkEllipsoidSDF.value(), "ellipsoid_link", 1,
            ignition::math::Vector3d(0.068, 0.058, 0.026),
            0u, sdf::GeometryType::EMPTY);
}
