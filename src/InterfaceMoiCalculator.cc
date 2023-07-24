/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "sdf/InterafaceMoiCalculator.hh"
#include "sdf/Mesh.hh"
#include "sdf/Element.hh"

using namespace sdf;

class InterfaceMoiCalculator::Implementation
{
  /// \brief Density of the mesh
  public: double density;

  /// \brief SDF mesh object 
  public: sdf::Mesh mesh;

  /// \brief SDF element pointer to <moi_calculator_params> tag. This can be used 
  /// to access custom params for the Mesh Intertia Caluclator 
  public: sdf::ElementPtr calculatorParams;
};

InterfaceMoiCalculator::InterfaceMoiCalculator(const double _density,
    const sdf::Mesh _mesh,
    const sdf::ElementPtr _calculatorParams)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->dataPtr->density = _density;
  this->dataPtr->mesh = _mesh;
  this->dataPtr->calculatorParams = _calculatorParams;
}

const double InterfaceMoiCalculator::Density() const
{
  return this->dataPtr->density;
}

void InterfaceMoiCalculator::SetDensity(const double _density)
{
  this->dataPtr->density = _density;
}

const sdf::Mesh &InterfaceMoiCalculator::Mesh() const
{
  return this->dataPtr->mesh;
}

void InterfaceMoiCalculator::SetMesh(sdf::Mesh &_mesh)
{
  this->dataPtr->mesh = _mesh;
}

const sdf::ElementPtr InterfaceMoiCalculator::MoiCalculatorParams() const
{
  return this->dataPtr->calculatorParams;
}

void InterfaceMoiCalculator::SetMoiCalculatorParams(sdf::ElementPtr _calculatorParams)
{
  this->dataPtr->calculatorParams = _calculatorParams;
}