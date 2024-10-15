/*
 * Copyright 2023 Open Source Robotics Foundation
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

#include <sdf/CustomInertiaCalcProperties.hh>
#include <sdf/Element.hh>

#include <gz/math/Vector3.hh>

/////////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, Construction)
{
  sdf::CustomInertiaCalcProperties customInertiaCaclProps;

  EXPECT_DOUBLE_EQ(customInertiaCaclProps.Density(), 1000.0);
  customInertiaCaclProps.SetDensity(1240.0);
  EXPECT_DOUBLE_EQ(customInertiaCaclProps.Density(), 1240.0);

  EXPECT_EQ(customInertiaCaclProps.AutoInertiaParams(), nullptr);
  sdf::ElementPtr autoInertiaParamsElem = sdf::ElementPtr(new sdf::Element);
  autoInertiaParamsElem->SetName("auto_inertia_params");
  customInertiaCaclProps.SetAutoInertiaParams(autoInertiaParamsElem);
  EXPECT_EQ(customInertiaCaclProps.AutoInertiaParams(),
    autoInertiaParamsElem);
  EXPECT_EQ(customInertiaCaclProps.AutoInertiaParams()->GetName(),
    autoInertiaParamsElem->GetName());

  EXPECT_EQ(customInertiaCaclProps.Mesh(), std::nullopt);
  sdf::Mesh mesh;
  mesh.SetScale(gz::math::Vector3d(2, 2, 2));
  customInertiaCaclProps.SetMesh(mesh);
  EXPECT_EQ(customInertiaCaclProps.Mesh()->Scale(), mesh.Scale());
}

/////////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, ConstructionWithMeshConstructor)
{
  double density = 1240.0;

   sdf::Mesh mesh;
   mesh.SetScale(gz::math::Vector3d(2, 2, 2));

   sdf::ElementPtr autoInertiaParamsElem = sdf::ElementPtr(new sdf::Element);
   autoInertiaParamsElem->SetName("auto_inertia_params");

   sdf::CustomInertiaCalcProperties customInertiaCalcProps(
     density,
     mesh,
     autoInertiaParamsElem
   );

  EXPECT_DOUBLE_EQ(customInertiaCalcProps.Density(), density);
  EXPECT_EQ(customInertiaCalcProps.Mesh()->Scale(), mesh.Scale());
  EXPECT_EQ(customInertiaCalcProps.AutoInertiaParams(),
    autoInertiaParamsElem);
}

/////////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, CopyConstructor)
{
  sdf::CustomInertiaCalcProperties customInertiaCalcProps;
  customInertiaCalcProps.SetDensity(1240.0);

  sdf::CustomInertiaCalcProperties customInertiaCalcProps2(
    customInertiaCalcProps);
  EXPECT_DOUBLE_EQ(customInertiaCalcProps2.Density(),
    customInertiaCalcProps.Density());
}

/////////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, CopyAssignmentOperator)
{
  sdf::CustomInertiaCalcProperties customInertiaCalcProps;
  customInertiaCalcProps.SetDensity(1240.0);

  sdf::CustomInertiaCalcProperties customInertiaCalcProps2;
  customInertiaCalcProps2 = customInertiaCalcProps;

  EXPECT_DOUBLE_EQ(customInertiaCalcProps2.Density(),
    customInertiaCalcProps.Density());
}

///////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, MoveConstructor)
{
  sdf::CustomInertiaCalcProperties customInertiaCalcProps;
  customInertiaCalcProps.SetDensity(1240.0);

  sdf::CustomInertiaCalcProperties customInertiaCalcProps2(
    std::move(customInertiaCalcProps));

  EXPECT_DOUBLE_EQ(customInertiaCalcProps2.Density(), 1240.0);
}

/////////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, MoveAssignmentOperator)
{
  sdf::CustomInertiaCalcProperties customInertiaCalcProps;
  customInertiaCalcProps.SetDensity(1240.0);

  sdf::CustomInertiaCalcProperties customInertiaCalcProps2;
  customInertiaCalcProps2 = std::move(customInertiaCalcProps);

  EXPECT_DOUBLE_EQ(customInertiaCalcProps2.Density(), 1240.0);
}

/////////////////////////////////////////////////
TEST(DOMCustomInertiaCalcProperties, CopyAssignmentAfterMove)
{
  sdf::CustomInertiaCalcProperties customInertiaCalcProps1;
  customInertiaCalcProps1.SetDensity(1240.0);

  sdf::CustomInertiaCalcProperties customInertiaCalcProps2;
  customInertiaCalcProps2.SetDensity(2710.0);

  EXPECT_DOUBLE_EQ(2710.0, customInertiaCalcProps2.Density());
  EXPECT_DOUBLE_EQ(1240.0, customInertiaCalcProps1.Density());

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::CustomInertiaCalcProperties tmp = std::move(customInertiaCalcProps1);
  customInertiaCalcProps1 = customInertiaCalcProps2;
  customInertiaCalcProps2 = tmp;

  EXPECT_DOUBLE_EQ(2710.0, customInertiaCalcProps1.Density());
  EXPECT_DOUBLE_EQ(1240.0, customInertiaCalcProps2.Density());
}
