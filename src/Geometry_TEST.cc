/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "sdf/Box.hh"
#include "sdf/Capsule.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Ellipsoid.hh"
#include "sdf/Geometry.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Sphere.hh"

/////////////////////////////////////////////////
TEST(DOMGeometry, Construction)
{
  sdf::Geometry geom;
  EXPECT_EQ(nullptr, geom.Element());
  EXPECT_EQ(sdf::GeometryType::EMPTY, geom.Type());

  geom.SetType(sdf::GeometryType::BOX);
  EXPECT_EQ(sdf::GeometryType::BOX, geom.Type());

  geom.SetType(sdf::GeometryType::CAPSULE);
  EXPECT_EQ(sdf::GeometryType::CAPSULE, geom.Type());

  geom.SetType(sdf::GeometryType::CYLINDER);
  EXPECT_EQ(sdf::GeometryType::CYLINDER, geom.Type());

  geom.SetType(sdf::GeometryType::ELLIPSOID);
  EXPECT_EQ(sdf::GeometryType::ELLIPSOID, geom.Type());

  geom.SetType(sdf::GeometryType::PLANE);
  EXPECT_EQ(sdf::GeometryType::PLANE, geom.Type());

  geom.SetType(sdf::GeometryType::SPHERE);
  EXPECT_EQ(sdf::GeometryType::SPHERE, geom.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, MoveConstructor)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);

  sdf::Geometry geometry2(std::move(geometry));
  EXPECT_EQ(sdf::GeometryType::BOX, geometry2.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, CopyConstructor)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);
  sdf::Box boxShape;
  boxShape.SetSize(ignition::math::Vector3d(1, 2, 3));
  geometry.SetBoxShape(boxShape);

  sdf::Geometry geometry2(geometry);
  EXPECT_EQ(sdf::GeometryType::BOX, geometry2.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, AssignmentOperator)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);
  sdf::Box boxShape;
  boxShape.SetSize(ignition::math::Vector3d(1, 2, 3));
  geometry.SetBoxShape(boxShape);

  sdf::Geometry geometry2;
  geometry2 = geometry;
  EXPECT_EQ(sdf::GeometryType::BOX, geometry2.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, MoveAssignmentOperator)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);
  sdf::Box boxShape;
  boxShape.SetSize(ignition::math::Vector3d(1, 2, 3));
  geometry.SetBoxShape(boxShape);

  sdf::Geometry geometry2;
  geometry2 = std::move(geometry);
  EXPECT_EQ(sdf::GeometryType::BOX, geometry2.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, CopyAssignmentAfterMove)
{
  sdf::Geometry geometry1;
  geometry1.SetType(sdf::GeometryType::BOX);

  sdf::Geometry geometry2;
  geometry2.SetType(sdf::GeometryType::SPHERE);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Geometry tmp = std::move(geometry1);
  geometry1 = geometry2;
  geometry2 = tmp;

  EXPECT_EQ(sdf::GeometryType::SPHERE, geometry1.Type());
  EXPECT_EQ(sdf::GeometryType::BOX, geometry2.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Load)
{
  sdf::Geometry geom;
  sdf::Errors errors;

  // Null element name
  errors = geom.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, geom.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = geom.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, geom.Element());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Box)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::BOX);

  sdf::Box boxShape;
  boxShape.SetSize(ignition::math::Vector3d(1, 2, 3));
  geom.SetBoxShape(boxShape);

  EXPECT_EQ(sdf::GeometryType::BOX, geom.Type());
  EXPECT_NE(nullptr, geom.BoxShape());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), geom.BoxShape()->Size());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Sphere)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::SPHERE);

  sdf::Sphere sphereShape;
  sphereShape.SetRadius(0.123);
  geom.SetSphereShape(sphereShape);

  EXPECT_EQ(sdf::GeometryType::SPHERE, geom.Type());
  EXPECT_NE(nullptr, geom.SphereShape());
  EXPECT_DOUBLE_EQ(0.123, geom.SphereShape()->Radius());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Capsule)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::CAPSULE);

  sdf::Capsule capsuleShape;
  capsuleShape.SetRadius(0.123);
  capsuleShape.SetLength(4.56);
  geom.SetCapsuleShape(capsuleShape);

  EXPECT_EQ(sdf::GeometryType::CAPSULE, geom.Type());
  EXPECT_NE(nullptr, geom.CapsuleShape());
  EXPECT_DOUBLE_EQ(0.123, geom.CapsuleShape()->Radius());
  EXPECT_DOUBLE_EQ(4.56, geom.CapsuleShape()->Length());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Cylinder)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::CYLINDER);

  sdf::Cylinder cylinderShape;
  cylinderShape.SetRadius(0.123);
  cylinderShape.SetLength(4.56);
  geom.SetCylinderShape(cylinderShape);

  EXPECT_EQ(sdf::GeometryType::CYLINDER, geom.Type());
  EXPECT_NE(nullptr, geom.CylinderShape());
  EXPECT_DOUBLE_EQ(0.123, geom.CylinderShape()->Radius());
  EXPECT_DOUBLE_EQ(4.56, geom.CylinderShape()->Length());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Ellipsoid)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::ELLIPSOID);

  sdf::Ellipsoid ellipsoidShape;
  const ignition::math::Vector3d expectedRadii(1, 2, 3);
  ellipsoidShape.SetRadii(expectedRadii);
  geom.SetEllipsoidShape(ellipsoidShape);

  EXPECT_EQ(sdf::GeometryType::ELLIPSOID, geom.Type());
  EXPECT_NE(nullptr, geom.EllipsoidShape());
  EXPECT_EQ(expectedRadii, geom.EllipsoidShape()->Radii());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Mesh)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::MESH);

  sdf::Mesh meshShape;
  meshShape.SetScale(ignition::math::Vector3d(1, 2, 3));
  meshShape.SetUri("banana");
  meshShape.SetSubmesh("orange");
  meshShape.SetCenterSubmesh(true);
  geom.SetMeshShape(meshShape);

  EXPECT_EQ(sdf::GeometryType::MESH, geom.Type());
  EXPECT_NE(nullptr, geom.MeshShape());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), geom.MeshShape()->Scale());
  EXPECT_EQ("banana", geom.MeshShape()->Uri());
  EXPECT_EQ("orange", geom.MeshShape()->Submesh());
  EXPECT_TRUE(geom.MeshShape()->CenterSubmesh());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Plane)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::PLANE);

  sdf::Plane planeShape;
  planeShape.SetNormal(ignition::math::Vector3d::UnitX);
  planeShape.SetSize(ignition::math::Vector2d(9, 8));
  geom.SetPlaneShape(planeShape);

  EXPECT_EQ(sdf::GeometryType::PLANE, geom.Type());
  EXPECT_NE(nullptr, geom.PlaneShape());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, geom.PlaneShape()->Normal());
  EXPECT_EQ(ignition::math::Vector2d(9, 8), geom.PlaneShape()->Size());
}
