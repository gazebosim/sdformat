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
#include <optional>

#include <gtest/gtest.h>
#include "sdf/Mesh.hh"
#include "test_utils.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/Types.hh"
#include "sdf/Error.hh"

#include <gz/math/Vector3.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Inertial.hh>

/////////////////////////////////////////////////
TEST(DOMMesh, Construction)
{
  sdf::Mesh mesh;
  EXPECT_EQ(nullptr, mesh.Element());

  EXPECT_EQ(std::string(), mesh.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::NONE, mesh.Optimization());
  EXPECT_EQ(nullptr, mesh.ConvexDecomposition());
  EXPECT_EQ(std::string(), mesh.FilePath());
  EXPECT_EQ(std::string(), mesh.Uri());
  EXPECT_EQ(std::string(), mesh.Submesh());
  EXPECT_TRUE(gz::math::Vector3d(1, 1, 1) == mesh.Scale());
  EXPECT_FALSE(mesh.CenterSubmesh());
}

/////////////////////////////////////////////////
TEST(DOMMesh, MoveConstructor)
{
  sdf::Mesh mesh;
  EXPECT_TRUE(mesh.SetOptimization("convex_decomposition"));
  mesh.SetUri("banana");
  mesh.SetSubmesh("watermelon");
  mesh.SetCenterSubmesh(true);
  mesh.SetScale({0.5, 0.6, 0.7});
  mesh.SetFilePath("/pear");

  sdf::ConvexDecomposition convexDecomp;
  EXPECT_EQ(nullptr, convexDecomp.Element());
  convexDecomp.SetMaxConvexHulls(10u);
  convexDecomp.SetVoxelResolution(100000u);
  mesh.SetConvexDecomposition(convexDecomp);

  sdf::Mesh mesh2(std::move(mesh));
  EXPECT_EQ("convex_decomposition", mesh2.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::CONVEX_DECOMPOSITION, mesh2.Optimization());
  EXPECT_EQ("banana", mesh2.Uri());
  EXPECT_EQ("watermelon", mesh2.Submesh());
  EXPECT_EQ(gz::math::Vector3d(0.5, 0.6, 0.7), mesh2.Scale());
  EXPECT_TRUE(mesh2.CenterSubmesh());
  EXPECT_EQ("/pear", mesh2.FilePath());

  auto convexDecomp2 = mesh2.ConvexDecomposition();
  ASSERT_NE(nullptr, convexDecomp2);
  EXPECT_EQ(10u, convexDecomp2->MaxConvexHulls());
  EXPECT_EQ(100000u, convexDecomp2->VoxelResolution());
}

/////////////////////////////////////////////////
TEST(DOMMesh, CopyConstructor)
{
  sdf::Mesh mesh;
  EXPECT_TRUE(mesh.SetOptimization("convex_hull"));
  mesh.SetUri("banana");
  mesh.SetSubmesh("watermelon");
  mesh.SetCenterSubmesh(true);
  mesh.SetScale({0.5, 0.6, 0.7});
  mesh.SetFilePath("/pear");

  sdf::Mesh mesh2(mesh);
  EXPECT_EQ("convex_hull", mesh2.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::CONVEX_HULL, mesh2.Optimization());
  EXPECT_EQ(nullptr, mesh2.ConvexDecomposition());
  EXPECT_EQ("banana", mesh2.Uri());
  EXPECT_EQ("watermelon", mesh2.Submesh());
  EXPECT_EQ(gz::math::Vector3d(0.5, 0.6, 0.7), mesh2.Scale());
  EXPECT_TRUE(mesh2.CenterSubmesh());
  EXPECT_EQ("/pear", mesh2.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMesh, CopyAssignmentOperator)
{
  sdf::Mesh mesh;
  EXPECT_TRUE(mesh.SetOptimization("convex_hull"));
  mesh.SetUri("banana");
  mesh.SetSubmesh("watermelon");
  mesh.SetCenterSubmesh(true);
  mesh.SetScale({0.5, 0.6, 0.7});
  mesh.SetFilePath("/pear");

  sdf::Mesh mesh2;
  mesh2 = mesh;
  EXPECT_EQ("convex_hull", mesh2.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::CONVEX_HULL, mesh2.Optimization());
  EXPECT_EQ(nullptr, mesh2.ConvexDecomposition());
  EXPECT_EQ("banana", mesh2.Uri());
  EXPECT_EQ("watermelon", mesh2.Submesh());
  EXPECT_EQ(gz::math::Vector3d(0.5, 0.6, 0.7), mesh2.Scale());
  EXPECT_TRUE(mesh2.CenterSubmesh());
  EXPECT_EQ("/pear", mesh2.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMesh, MoveAssignmentOperator)
{
  sdf::Mesh mesh;
  EXPECT_TRUE(mesh.SetOptimization("convex_hull"));
  mesh.SetUri("banana");
  mesh.SetSubmesh("watermelon");
  mesh.SetCenterSubmesh(true);
  mesh.SetScale({0.5, 0.6, 0.7});
  mesh.SetFilePath("/pear");

  sdf::Mesh mesh2;
  mesh2 = std::move(mesh);
  EXPECT_EQ("convex_hull", mesh2.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::CONVEX_HULL, mesh2.Optimization());
  EXPECT_EQ(nullptr, mesh2.ConvexDecomposition());
  EXPECT_EQ("banana", mesh2.Uri());
  EXPECT_EQ("watermelon", mesh2.Submesh());
  EXPECT_EQ(gz::math::Vector3d(0.5, 0.6, 0.7), mesh2.Scale());
  EXPECT_TRUE(mesh2.CenterSubmesh());
  EXPECT_EQ("/pear", mesh2.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMesh, CopyAssignmentAfterMove)
{
  sdf::Mesh mesh1;
  mesh1.SetUri("banana");

  sdf::Mesh mesh2;
  mesh2.SetUri("watermelon");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Mesh tmp = std::move(mesh1);
  mesh1 = mesh2;
  mesh2 = tmp;

  EXPECT_EQ("watermelon", mesh1.Uri());
  EXPECT_EQ("banana", mesh2.Uri());
}

/////////////////////////////////////////////////
TEST(DOMMesh, Set)
{
  sdf::Mesh mesh;
  EXPECT_EQ(nullptr, mesh.Element());

  EXPECT_EQ(std::string(), mesh.OptimizationStr());
  EXPECT_TRUE(mesh.SetOptimization("convex_hull"));
  EXPECT_EQ("convex_hull", mesh.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::CONVEX_HULL, mesh.Optimization());
  mesh.SetOptimization(sdf::MeshOptimization::CONVEX_DECOMPOSITION);
  EXPECT_EQ("convex_decomposition", mesh.OptimizationStr());
  EXPECT_EQ(sdf::MeshOptimization::CONVEX_DECOMPOSITION,
            mesh.Optimization());
  // check invalid inputs
  EXPECT_FALSE(mesh.SetOptimization("invalid"));
  {
    auto invalidMeshOpt = static_cast<sdf::MeshOptimization>(99);
    mesh.SetOptimization(invalidMeshOpt);
    EXPECT_EQ(invalidMeshOpt, mesh.Optimization());
    EXPECT_EQ("", mesh.OptimizationStr());
  }

  sdf::ConvexDecomposition convexDecomp;
  convexDecomp.SetMaxConvexHulls(10u);
  convexDecomp.SetVoxelResolution(200000u);
  mesh.SetConvexDecomposition(convexDecomp);
  ASSERT_NE(nullptr, mesh.ConvexDecomposition());
  EXPECT_EQ(10u, mesh.ConvexDecomposition()->MaxConvexHulls());
  EXPECT_EQ(200000u, mesh.ConvexDecomposition()->VoxelResolution());

  EXPECT_EQ(std::string(), mesh.Uri());
  mesh.SetUri("http://myuri.com");
  EXPECT_EQ("http://myuri.com", mesh.Uri());

  EXPECT_EQ(std::string(), mesh.Submesh());
  mesh.SetSubmesh("my_submesh");
  EXPECT_EQ("my_submesh", mesh.Submesh());

  EXPECT_TRUE(gz::math::Vector3d(1, 1, 1) == mesh.Scale());
  mesh.SetScale(gz::math::Vector3d(0.2, 1.4, 7.8));
  EXPECT_TRUE(gz::math::Vector3d(0.2, 1.4, 7.8) == mesh.Scale());

  EXPECT_FALSE(mesh.CenterSubmesh());
  mesh.SetCenterSubmesh(true);
  EXPECT_TRUE(mesh.CenterSubmesh());

  EXPECT_EQ(std::string(), mesh.FilePath());
  mesh.SetFilePath("/mypath");
  EXPECT_EQ("/mypath", mesh.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMesh, Load)
{
  sdf::Mesh mesh;
  sdf::ConvexDecomposition convexDecomp;
  sdf::Errors errors;

  // Null element name
  errors = mesh.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, mesh.Element());

  errors = convexDecomp.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, convexDecomp.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = mesh.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, mesh.Element());

  errors = convexDecomp.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, convexDecomp.Element());

  // Missing <uri> element
  sdf->SetName("mesh");
  errors = mesh.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <uri>"));
  EXPECT_NE(nullptr, mesh.Element());
}

/////////////////////////////////////////////////
TEST(DOMMesh, CalcualteInertial)
{
  sdf::Mesh mesh;
  sdf::ParserConfig config;
  sdf::Errors errors;
  sdf::CustomInertiaCalcProperties inertiaCalcProps;

  double density = 0;
  sdf::ElementPtr autoInertiaParamsElem(new sdf::Element());

  // Test Lambda function for registering as custom inertia calculator
  auto customMeshInertiaCalculator = [](
    sdf::Errors &_errors,
    const sdf::CustomInertiaCalcProperties &_inertiaProps
  ) -> std::optional<gz::math::Inertiald>
  {
    if (_inertiaProps.Density() <= 0)
    {
      _errors.push_back(
        {sdf::ErrorCode::LINK_INERTIA_INVALID,
        "Inertia is invalid"});
      return std::nullopt;
    }

    gz::math::Inertiald meshInerial;

    meshInerial.SetMassMatrix(
      gz::math::MassMatrix3d(
        1.0,
        gz::math::Vector3d::One,
        gz::math::Vector3d::Zero
      )
    );

    return meshInerial;
  };

  config.RegisterCustomInertiaCalc(customMeshInertiaCalculator);

  auto meshInertial = mesh.CalculateInertial(errors,
  density, autoInertiaParamsElem, config);
  ASSERT_FALSE(errors.empty());
  ASSERT_EQ(meshInertial, std::nullopt);

  density = 1240.0;
  sdf::Errors errors2;
  auto meshInertial2 = mesh.CalculateInertial(errors2,
  density, autoInertiaParamsElem, config);
  ASSERT_TRUE(errors2.empty());
  EXPECT_DOUBLE_EQ(meshInertial2->MassMatrix().Mass(), 1.0);
  EXPECT_EQ(meshInertial2->MassMatrix().DiagonalMoments(),
    gz::math::Vector3d::One);
}

/////////////////////////////////////////////////
TEST(DOMMesh, CalculateInertiaWithEmptyFilePath)
{
  sdf::Mesh mesh;
  sdf::ParserConfig config;
  sdf::Errors errors;
  sdf::CustomInertiaCalcProperties inertiaCalcProps;

  double density = 0;
  sdf::ElementPtr autoInertiaParamsElem(new sdf::Element());

  // Test Lambda function for registering as custom inertia calculator
  auto customMeshInertiaCalculator = [](
    sdf::Errors &_errors,
    const sdf::CustomInertiaCalcProperties &_inertiaProps
  ) -> std::optional<gz::math::Inertiald>
  {
    if (_inertiaProps.Density() <= 0)
    {
      _errors.push_back(
        {sdf::ErrorCode::LINK_INERTIA_INVALID,
        "Inertia is invalid"});
      return std::nullopt;
    }

    gz::math::Inertiald meshInerial;

    meshInerial.SetMassMatrix(
      gz::math::MassMatrix3d(
        1.0,
        gz::math::Vector3d::One,
        gz::math::Vector3d::Zero
      )
    );

    return meshInerial;
  };

  config.RegisterCustomInertiaCalc(customMeshInertiaCalculator);

  auto meshInertial = mesh.CalculateInertial(errors,
    density, autoInertiaParamsElem, config);
  ASSERT_FALSE(errors.empty());
  ASSERT_EQ(meshInertial, std::nullopt);
}

/////////////////////////////////////////////////
TEST(DOMMesh, ToElement)
{
  sdf::Mesh mesh;

  EXPECT_TRUE(mesh.SetOptimization("convex_decomposition"));
  mesh.SetUri("mesh-uri");
  mesh.SetScale(gz::math::Vector3d(1, 2, 3));
  mesh.SetSubmesh("submesh");
  mesh.SetCenterSubmesh(false);

  sdf::ConvexDecomposition convexDecomp;
  convexDecomp.SetMaxConvexHulls(10u);
  convexDecomp.SetVoxelResolution(300000u);
  mesh.SetConvexDecomposition(convexDecomp);

  sdf::ElementPtr elem = mesh.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Mesh mesh2;
  mesh2.Load(elem);

  EXPECT_EQ(mesh.OptimizationStr(), mesh2.OptimizationStr());
  EXPECT_EQ(mesh.Optimization(), mesh2.Optimization());
  EXPECT_EQ(mesh.Uri(), mesh2.Uri());
  EXPECT_EQ(mesh.Scale(), mesh2.Scale());
  EXPECT_EQ(mesh.Submesh(), mesh2.Submesh());
  EXPECT_EQ(mesh.CenterSubmesh(), mesh2.CenterSubmesh());
  ASSERT_NE(nullptr, mesh2.ConvexDecomposition());
  EXPECT_EQ(10u, mesh2.ConvexDecomposition()->MaxConvexHulls());
  EXPECT_EQ(300000u, mesh2.ConvexDecomposition()->VoxelResolution());
}

/////////////////////////////////////////////////
TEST(DOMMesh, ToElementErrorOutput)
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

  sdf::Mesh mesh;
  sdf::Errors errors;

  EXPECT_TRUE(mesh.SetOptimization("convex_hull"));
  mesh.SetUri("mesh-uri");
  mesh.SetScale(gz::math::Vector3d(1, 2, 3));
  mesh.SetSubmesh("submesh");
  mesh.SetCenterSubmesh(false);

  sdf::ElementPtr elem = mesh.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Mesh mesh2;
  errors = mesh2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(mesh.OptimizationStr(), mesh2.OptimizationStr());
  EXPECT_EQ(mesh.Optimization(), mesh2.Optimization());
  EXPECT_EQ(nullptr, mesh2.ConvexDecomposition());
  EXPECT_EQ(mesh.Uri(), mesh2.Uri());
  EXPECT_EQ(mesh.Scale(), mesh2.Scale());
  EXPECT_EQ(mesh.Submesh(), mesh2.Submesh());
  EXPECT_EQ(mesh.CenterSubmesh(), mesh2.CenterSubmesh());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
