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

#ifndef SDF_USD_USD_PARSER_USDTRANSFORMS_HH_
#define SDF_USD_USD_PARSER_USDTRANSFORMS_HH_

#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/utils/ImplPtr.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/usd/Export.hh"
#include "sdf/usd/UsdError.hh"
#include "USDData.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief This class stores the transforms of a schema
    /// This might contain scale, translate or rotation operations
    /// The booleans are used to check if there is a transform defined
    /// in the schema
    class IGNITION_SDFORMAT_USD_VISIBLE UDSTransforms
    {
      /// \brief Default constructor
      public: UDSTransforms();

      /// \brief Translate
      /// \return A 3D vector with the translation
      public: const ignition::math::Vector3d Translation() const;

      /// \brief Scale
      /// \return A 3D vector with the scale
      public: const ignition::math::Vector3d Scale() const;

      /// \brief Get the Rotation
      /// \return Return The rotation, if one exists. If no rotation exists,
      /// std::nullopt is returned
      public: const std::optional<ignition::math::Quaterniond> Rotation() const;

      /// \brief Set translate
      /// \param[in] _translate Translate to set
      public: void SetTranslation(const ignition::math::Vector3d &_translate);

      /// \brief Set scale
      /// \param[in] _scale Scale to set
      public: void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Set rotation
      /// \param[in] _q Quaternion that defines the rotation
      public: void SetRotation(const ignition::math::Quaterniond &_q);

      /// \brief Private data pointer.
      IGN_UTILS_IMPL_PTR(dataPtr)
    };

    /// \brief This function gets the transform from a prim to the specified
    /// _schemaToStop variable
    /// \param[in] _prim Initial prim to read the transform
    /// \param[in] _usdData USDData structure to get info about the prim, for
    /// example: metersperunit
    /// \param[out] _pose Pose of the prim. From _prim to _schemaToStop.
    /// \param[out] _scale The scale of the prim
    /// \param[in] _schemaToStop Name of the prim where the loop will stop
    /// reading transforms
    void IGNITION_SDFORMAT_USD_VISIBLE GetTransform(
      const pxr::UsdPrim &_prim,
      const USDData &_usdData,
      ignition::math::Pose3d &_pose,
      ignition::math::Vector3d &_scale,
      const std::string &_schemaToStop);

    /// \brief Read the usd prim transforms. Scale, rotation or transform might
    /// be defined as float or doubles
    /// \param[in] _prim Prim where the transforms are read
    /// \return A USDTransforms class with all the transforms related to
    /// the prim
    UDSTransforms IGNITION_SDFORMAT_USD_VISIBLE ParseUSDTransform(
      const pxr::UsdPrim &_prim);
}
}
}
#endif
