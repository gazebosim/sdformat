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

#include <optional>

#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/Mesh.hh"
#include "sdf/Element.hh"

#include <gz/utils/ImplPtr.hh>

using namespace sdf;

class CustomInertiaCalcProperties::Implementation
{
  /// \brief Density of the mesh. 1000 kg/m^3 by default
  public: double density{1000.0};

  /// \brief Optional SDF mesh object. Default is std::nullopt
  public: std::optional<sdf::Mesh> mesh{std::nullopt};

  /// \brief SDF element pointer to <auto_inertia_params> tag.
  /// This can be used to access custom params for the
  /// Inertia Caluclator
  public: sdf::ElementPtr inertiaCalculatorParams{nullptr};
};

/////////////////////////////////////////////////
CustomInertiaCalcProperties::CustomInertiaCalcProperties()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

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
const std::optional<sdf::Mesh> &CustomInertiaCalcProperties::Mesh() const
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
void CustomInertiaCalcProperties::SetAutoInertiaParams(
    sdf::ElementPtr _autoInertiaParamsElem)
{
  this->dataPtr->inertiaCalculatorParams = _autoInertiaParamsElem;
}
