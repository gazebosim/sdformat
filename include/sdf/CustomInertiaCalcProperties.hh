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

#ifndef SDF_CUSTOM_INERTIA_CALC_PROPERTIES_HH_
#define SDF_CUSTOM_INERTIA_CALC_PROPERTIES_HH_

#include <optional>

#include <gz/utils/ImplPtr.hh>
#include <gz/math/Inertial.hh>

#include "sdf/Element.hh"
#include "sdf/Mesh.hh"
#include "sdf/config.hh"
#include "sdf/Types.hh"
#include "sdf/Error.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{

// Forward Declarations
class Mesh;

class SDFORMAT_VISIBLE CustomInertiaCalcProperties
{
  /// \brief Default Constructor
  public: CustomInertiaCalcProperties();

  /// \brief Constructor with mesh properties
  /// \param[in] _density Double density value
  /// \param[in] _mesh sdf::Mesh object
  /// \param[in] _calculatorParams sdf::ElementPtr for calculator params element
  public: CustomInertiaCalcProperties(const double _density,
              const sdf::Mesh _mesh,
              const sdf::ElementPtr _calculatorParams);

  /// \brief Get the density of the mesh.
  /// \return Double density of the mesh.
  public: double Density() const;

  /// \brief Function to set the density of the interface object
  /// \param[in] _density Double density value
  public: void SetDensity(double _density);

  /// \brief Get the reference to the mesh object being used.
  /// \return Reference to the sdf::Mesh object.
  public: const std::optional<sdf::Mesh> &Mesh() const;

  /// \brief Function to set the mesh object
  /// \param[in] _mesh sdf::Mesh object
  public: void SetMesh(sdf::Mesh &_mesh);

  /// \brief Get the reference to the <auto_inertia_params> sdf element.
  /// User defined calculator params can be accessed through this element
  /// \return sdf::ElementPtr for the tag
  public: const sdf::ElementPtr AutoInertiaParams() const;

  /// \brief Function to set the calculator params sdf element object
  /// \param[in] _autoInertiaParamsElem sdf::ElementPtr for calculator params
  public: void SetAutoInertiaParams(sdf::ElementPtr _autoInertiaParamsElem);

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR(dataPtr)
};

using CustomInertiaCalculator =
    std::function<std::optional<gz::math::Inertiald>(sdf::Errors &,
        const sdf::CustomInertiaCalcProperties &)>;
}
}

#endif
