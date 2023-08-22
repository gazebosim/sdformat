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
#include "sdf/Heightmap.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Polyline.hh"
#include "sdf/Sphere.hh"
#include "test_utils.hh"

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

  geom.SetType(sdf::GeometryType::POLYLINE);
  EXPECT_EQ(sdf::GeometryType::POLYLINE, geom.Type());
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
  boxShape.SetSize(gz::math::Vector3d(1, 2, 3));
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
  boxShape.SetSize(gz::math::Vector3d(1, 2, 3));
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
  boxShape.SetSize(gz::math::Vector3d(1, 2, 3));
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
  boxShape.SetSize(gz::math::Vector3d(1, 2, 3));
  geom.SetBoxShape(boxShape);

  EXPECT_EQ(sdf::GeometryType::BOX, geom.Type());
  EXPECT_NE(nullptr, geom.BoxShape());
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), geom.BoxShape()->Size());
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
  const gz::math::Vector3d expectedRadii(1, 2, 3);
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
  meshShape.SetScale(gz::math::Vector3d(1, 2, 3));
  meshShape.SetUri("banana");
  meshShape.SetSubmesh("orange");
  meshShape.SetCenterSubmesh(true);
  geom.SetMeshShape(meshShape);

  EXPECT_EQ(sdf::GeometryType::MESH, geom.Type());
  EXPECT_NE(nullptr, geom.MeshShape());
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), geom.MeshShape()->Scale());
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
  planeShape.SetNormal(gz::math::Vector3d::UnitX);
  planeShape.SetSize(gz::math::Vector2d(9, 8));
  geom.SetPlaneShape(planeShape);

  EXPECT_EQ(sdf::GeometryType::PLANE, geom.Type());
  EXPECT_NE(nullptr, geom.PlaneShape());
  EXPECT_EQ(gz::math::Vector3d::UnitX, geom.PlaneShape()->Normal());
  EXPECT_EQ(gz::math::Vector2d(9, 8), geom.PlaneShape()->Size());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Polyline)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::POLYLINE);

  sdf::Polyline polylineShape1;
  polylineShape1.SetHeight(1.2);
  EXPECT_TRUE(polylineShape1.AddPoint({3.4, 5.6}));
  EXPECT_TRUE(polylineShape1.AddPoint({7.8, 9.0}));

  sdf::Polyline polylineShape2;
  polylineShape2.SetHeight(2.1);
  EXPECT_TRUE(polylineShape2.AddPoint({4.3, 6.5}));
  EXPECT_TRUE(polylineShape2.AddPoint({8.7, 0.9}));

  geom.SetPolylineShape({polylineShape1, polylineShape2});

  EXPECT_EQ(sdf::GeometryType::POLYLINE, geom.Type());
  EXPECT_FALSE(geom.PolylineShape().empty());
  auto polylineShape = geom.PolylineShape();
  EXPECT_EQ(2u, polylineShape.size());
  EXPECT_DOUBLE_EQ(1.2, polylineShape[0].Height());
  ASSERT_EQ(2u, polylineShape[0].PointCount());
  EXPECT_EQ(gz::math::Vector2d(3.4, 5.6), polylineShape[0].Points()[0]);
  EXPECT_EQ(gz::math::Vector2d(7.8, 9.0), polylineShape[0].Points()[1]);
  EXPECT_DOUBLE_EQ(2.1, polylineShape[1].Height());
  ASSERT_EQ(2u, polylineShape[1].PointCount());
  EXPECT_EQ(gz::math::Vector2d(4.3, 6.5), polylineShape[1].Points()[0]);
  EXPECT_EQ(gz::math::Vector2d(8.7, 0.9), polylineShape[1].Points()[1]);
}

/////////////////////////////////////////////////
TEST(DOMGeometry, CalculateInertial)
{
  sdf::Geometry geom;

  // Density of Aluminimum
  double density = 2170.0;
  double expectedMass;
  gz::math::MassMatrix3d expectedMassMat;
  gz::math::Inertiald expectedInertial;

  // Box
  {
    sdf::Box box;
    double l = 2;
    double w = 2;
    double h = 2;
    box.SetSize(gz::math::Vector3d(l, w, h));

    expectedMass = box.Shape().Volume() * density;
    double ixx = (1.0/12.0) * expectedMass * (w*w + h*h);
    double iyy = (1.0/12.0) * expectedMass * (l*l + h*h);
    double izz = (1.0/12.0) * expectedMass * (l*l + w*w);

    expectedMassMat.SetMass(expectedMass);
    expectedMassMat.SetDiagonalMoments(gz::math::Vector3d(ixx, iyy, izz));
    expectedMassMat.SetOffDiagonalMoments(gz::math::Vector3d::Zero);

    expectedInertial.SetMassMatrix(expectedMassMat);
    expectedInertial.SetPose(gz::math::Pose3d::Zero);

    geom.SetType(sdf::GeometryType::BOX);
    geom.SetBoxShape(box);
    auto boxInertial = geom.CalculateInertial(density);

    ASSERT_NE(std::nullopt, boxInertial);
    EXPECT_EQ(expectedInertial, *boxInertial);
    EXPECT_EQ(expectedInertial.MassMatrix(), expectedMassMat);
    EXPECT_EQ(expectedInertial.Pose(), boxInertial->Pose());
  }

  // Capsule
  {
    sdf::Capsule capsule;
    double l = 2.0;
    double r = 0.1;
    capsule.SetLength(l);
    capsule.SetRadius(r);

    expectedMass = capsule.Shape().Volume() * density;
    const double cylinderVolume = GZ_PI * r*r * l;
    const double sphereVolume = GZ_PI * 4. / 3. * r*r*r;
    const double volume = cylinderVolume + sphereVolume;
    const double cylinderMass = expectedMass * cylinderVolume / volume;
    const double sphereMass = expectedMass * sphereVolume / volume;
    double ixxIyy = (1/12.0) * cylinderMass * (3*r*r + l*l)
      + sphereMass * (0.4*r*r + 0.375*r*l + 0.25*l*l);
    double izz = r*r * (0.5 * cylinderMass + 0.4 * sphereMass);

    expectedMassMat.SetMass(expectedMass);
    expectedMassMat.SetDiagonalMoments(gz::math::Vector3d(ixxIyy, ixxIyy, izz));
    expectedMassMat.SetOffDiagonalMoments(gz::math::Vector3d::Zero);

    expectedInertial.SetMassMatrix(expectedMassMat);
    expectedInertial.SetPose(gz::math::Pose3d::Zero);

    geom.SetType(sdf::GeometryType::CAPSULE);
    geom.SetCapsuleShape(capsule);
    auto capsuleInertial = geom.CalculateInertial(density);

    ASSERT_NE(std::nullopt, capsuleInertial);
    EXPECT_EQ(expectedInertial, *capsuleInertial);
    EXPECT_EQ(expectedInertial.MassMatrix(), expectedMassMat);
    EXPECT_EQ(expectedInertial.Pose(), capsuleInertial->Pose());
  }

  // Cylinder
  {
    sdf::Cylinder cylinder;
    double l = 2.0;
    double r = 0.1;

    cylinder.SetLength(l);
    cylinder.SetRadius(r);

    expectedMass = cylinder.Shape().Volume() * density;
    double ixxIyy = (1/12.0) * expectedMass * (3*r*r + l*l);
    double izz = 0.5 * expectedMass * r * r;

    expectedMassMat.SetMass(expectedMass);
    expectedMassMat.SetDiagonalMoments(gz::math::Vector3d(ixxIyy, ixxIyy, izz));
    expectedMassMat.SetOffDiagonalMoments(gz::math::Vector3d::Zero);

    expectedInertial.SetMassMatrix(expectedMassMat);
    expectedInertial.SetPose(gz::math::Pose3d::Zero);

    geom.SetType(sdf::GeometryType::CYLINDER);
    geom.SetCylinderShape(cylinder);
    auto cylinderInertial = geom.CalculateInertial(density);

    ASSERT_NE(std::nullopt, cylinderInertial);
    EXPECT_EQ(expectedInertial, *cylinderInertial);
    EXPECT_EQ(expectedInertial.MassMatrix(), expectedMassMat);
    EXPECT_EQ(expectedInertial.Pose(), cylinderInertial->Pose());
  }

  // Ellipsoid
  {
    sdf::Ellipsoid ellipsoid;

    double a = 1.0;
    double b = 10.0;
    double c = 100.0;

    ellipsoid.SetRadii(gz::math::Vector3d(a, b, c));

    expectedMass = ellipsoid.Shape().Volume() * density;
    double ixx = (expectedMass / 5.0) * (b*b + c*c);
    double iyy = (expectedMass / 5.0) * (a*a + c*c);
    double izz = (expectedMass / 5.0) * (a*a + b*b);

    expectedMassMat.SetMass(expectedMass);
    expectedMassMat.SetDiagonalMoments(gz::math::Vector3d(ixx, iyy, izz));
    expectedMassMat.SetOffDiagonalMoments(gz::math::Vector3d::Zero);

    expectedInertial.SetMassMatrix(expectedMassMat);
    expectedInertial.SetPose(gz::math::Pose3d::Zero);

    geom.SetType(sdf::GeometryType::ELLIPSOID);
    geom.SetEllipsoidShape(ellipsoid);
    auto ellipsoidInertial = geom.CalculateInertial(density);

    ASSERT_NE(std::nullopt, ellipsoidInertial);
    EXPECT_EQ(expectedInertial, *ellipsoidInertial);
    EXPECT_EQ(expectedInertial.MassMatrix(), expectedMassMat);
    EXPECT_EQ(expectedInertial.Pose(), ellipsoidInertial->Pose());
  }

  // Sphere
  {
    sdf::Sphere sphere;
    double r = 0.1;

    sphere.SetRadius(r);

    expectedMass = sphere.Shape().Volume() * density;
    double ixxIyyIzz = 0.4 * expectedMass * r * r;

    expectedMassMat.SetMass(expectedMass);
    expectedMassMat.SetDiagonalMoments(
      gz::math::Vector3d(ixxIyyIzz, ixxIyyIzz, ixxIyyIzz));
    expectedMassMat.SetOffDiagonalMoments(gz::math::Vector3d::Zero);

    expectedInertial.SetMassMatrix(expectedMassMat);
    expectedInertial.SetPose(gz::math::Pose3d::Zero);

    geom.SetType(sdf::GeometryType::SPHERE);
    geom.SetSphereShape(sphere);
    auto sphereInertial = geom.CalculateInertial(density);

    ASSERT_NE(std::nullopt, sphereInertial);
    EXPECT_EQ(expectedInertial, *sphereInertial);
    EXPECT_EQ(expectedInertial.MassMatrix(), expectedMassMat);
    EXPECT_EQ(expectedInertial.Pose(), sphereInertial->Pose());
  }
}

/////////////////////////////////////////////////
TEST(DOMGeometry, ToElement)
{
  // Box
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::BOX);
    sdf::Box box;
    geom.SetBoxShape(box);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_NE(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Capsule
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::CAPSULE);
    sdf::Capsule capsule;
    geom.SetCapsuleShape(capsule);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_NE(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Cylinder
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::CYLINDER);
    sdf::Cylinder cylinder;
    geom.SetCylinderShape(cylinder);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_NE(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Ellipsoid
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::ELLIPSOID);
    sdf::Ellipsoid ellipsoid;
    geom.SetEllipsoidShape(ellipsoid);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_NE(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Sphere
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::SPHERE);
    sdf::Sphere sphere;
    geom.SetSphereShape(sphere);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_NE(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Plane
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::PLANE);
    sdf::Plane plane;
    geom.SetPlaneShape(plane);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_NE(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Mesh
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::MESH);
    sdf::Mesh mesh;
    geom.SetMeshShape(mesh);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_NE(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Heightmap
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::HEIGHTMAP);
    sdf::Heightmap heightmap;
    geom.SetHeightmapShape(heightmap);

    sdf::ElementPtr elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_NE(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Polyline
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::POLYLINE);
    sdf::Polyline polyline;
    geom.SetPolylineShape({polyline});

    auto elem = geom.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    geom2.Load(elem);

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_FALSE(geom2.PolylineShape().empty());
  }
}

/////////////////////////////////////////////////
TEST(DOMGeometry, ToElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  sdf::Errors errors;

  // Box
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::BOX);
    sdf::Box box;
    geom.SetBoxShape(box);

    sdf::ElementPtr elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_NE(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Capsule
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::CAPSULE);
    sdf::Capsule capsule;
    geom.SetCapsuleShape(capsule);

    sdf::ElementPtr elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_NE(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Cylinder
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::CYLINDER);
    sdf::Cylinder cylinder;
    geom.SetCylinderShape(cylinder);

    sdf::ElementPtr elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_NE(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Ellipsoid
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::ELLIPSOID);
    sdf::Ellipsoid ellipsoid;
    geom.SetEllipsoidShape(ellipsoid);

    sdf::ElementPtr elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_NE(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Sphere
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::SPHERE);
    sdf::Sphere sphere;
    geom.SetSphereShape(sphere);

    sdf::ElementPtr elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_NE(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Plane
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::PLANE);
    sdf::Plane plane;
    geom.SetPlaneShape(plane);

    sdf::ElementPtr elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_NE(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Mesh
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::MESH);
    sdf::Mesh mesh;
    geom.SetMeshShape(mesh);

    sdf::ElementPtr elem = geom.ToElement(errors);
    // Required uri is not set so an Error is expected
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Empty string used when setting a required parameter. Key[uri]"));
    errors.clear();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_NE(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Heightmap
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::HEIGHTMAP);
    sdf::Heightmap heightmap;
    geom.SetHeightmapShape(heightmap);

    sdf::ElementPtr elem = geom.ToElement(errors);
    // Required uri is not set so an Error is expected
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Empty string used when setting a required parameter. Key[uri]"));
    errors.clear();
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_NE(nullptr, geom2.HeightmapShape());
    EXPECT_TRUE(geom2.PolylineShape().empty());
  }

  // Polyline
  {
    sdf::Geometry geom;

    geom.SetType(sdf::GeometryType::POLYLINE);
    sdf::Polyline polyline;
    geom.SetPolylineShape({polyline});

    auto elem = geom.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Geometry geom2;
    errors = geom2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(geom.Type(), geom2.Type());
    EXPECT_EQ(nullptr, geom2.BoxShape());
    EXPECT_EQ(nullptr, geom2.CapsuleShape());
    EXPECT_EQ(nullptr, geom2.CylinderShape());
    EXPECT_EQ(nullptr, geom2.EllipsoidShape());
    EXPECT_EQ(nullptr, geom2.SphereShape());
    EXPECT_EQ(nullptr, geom2.PlaneShape());
    EXPECT_EQ(nullptr, geom2.MeshShape());
    EXPECT_EQ(nullptr, geom2.HeightmapShape());
    EXPECT_FALSE(geom2.PolylineShape().empty());
  }

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
