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

#ifndef SDF_INTERFACE_MOI_CALCULATOR_HH_
#define SDF_INTERFACE_MOI_CALCULATOR_HH_

#include <gz/utils/ImplPtr.hh>
#include <gz/math/Inertial.hh>

#include "sdf/Element.hh"
#include "sdf/Mesh.hh"
#include "sdf/config.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
  
// Forward Declarations
class Mesh;

class SDFORMAT_VISIBLE InterfaceMoiCalculator
{
  /// \brief Constructor
  /// \param[in] _density Double density value
  /// \param[in] _mesh sdf::Mesh object
  /// \param[in] _calculatorParamElem sdf::ElementPtr for calculator params element
  public: InterfaceMoiCalculator(const double _density,
              const sdf::Mesh _mesh,
              const sdf::ElementPtr _calculatorParams);
  
  /// \brief Get the density of the mesh.
  /// \return Double density of the mesh.
  public: const double Density() const;

  /// \brief Function to set the density of the interface object 
  /// \param[in] _density Double density value
  public: void SetDensity(const double _density);

  /// \brief Get the reference to the mesh oject being used.
  /// \return Reference to the sdf::Mesh object.
  public: const sdf::Mesh &Mesh() const;

  /// \brief Function to set the mesh object 
  /// \param[in] _mesh sdf::Mesh object
  public: void SetMesh(sdf::Mesh &_mesh);

  /// \brief Get the reference to the <moi_calculator_params> sdf element.
  /// User defined custom calculator params can be accessed through this element 
  /// \return sdf::ElementPtr for the tag
  public: const sdf::ElementPtr MoiCalculatorParams() const;

  /// \brief Function to set the calculator params sdf element object 
  /// \param[in] _calculatorParamElem sdf::ElementPtr for calculator params element
  public: void SetMoiCalculatorParams(sdf::ElementPtr _calculatorParamElem);

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR(dataPtr)
};

using CustomMOICalculator =
    std::function<std::optional<gz::math::Inertiald>(sdf::Errors &, 
        const sdf::InterfaceMoiCalculator &)>;
}
}

#endif