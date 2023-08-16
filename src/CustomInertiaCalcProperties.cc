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

#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/Mesh.hh"
#include "sdf/Element.hh"

using namespace sdf;

class CustomInertiaCalcProperties::Implementation
{
  /// \brief Density of the mesh
  public: double density;

  /// \brief SDF mesh object 
  public: sdf::Mesh mesh;

  /// \brief SDF element pointer to <moi_calculator_params> tag. This can be used 
  /// to access custom params for the Mesh Intertia Caluclator 
  public: sdf::ElementPtr inertiaCalculatorParams;
};

/////////////////////////////////////////////////
CustomInertiaCalcProperties::CustomInertiaCalcProperties(const double _density,
    const sdf::Mesh _mesh,
    const sdf::ElementPtr _calculatorParams)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->density = _density;
  this->dataPtr->mesh = _mesh;
  this->dataPtr->inertiaCalculatorParams = _calculatorParams;
}

/////////////////////////////////////////////////
double CustomInertiaCalcProperties::Density() const
{
  return this->dataPtr->density;
}

/////////////////////////////////////////////////
void CustomInertiaCalcProperties::SetDensity(double _density)
{
  this->dataPtr->density = _density;
}

/////////////////////////////////////////////////
const sdf::Mesh &CustomInertiaCalcProperties::Mesh() const
{
  return this->dataPtr->mesh;
}

/////////////////////////////////////////////////
void CustomInertiaCalcProperties::SetMesh(sdf::Mesh &_mesh)
{
  this->dataPtr->mesh = _mesh;
}

/////////////////////////////////////////////////
const sdf::ElementPtr CustomInertiaCalcProperties::AutoInertiaParams() const
{
  return this->dataPtr->inertiaCalculatorParams;
}

/////////////////////////////////////////////////
void CustomInertiaCalcProperties::SetAutoInertiaParams(sdf::ElementPtr _autoInertiaParams)
{
  this->dataPtr->inertiaCalculatorParams = _autoInertiaParams;
}