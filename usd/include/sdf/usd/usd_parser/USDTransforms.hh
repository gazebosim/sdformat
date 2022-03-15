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

#ifndef SDF_USD_USD_PARSER_UTILS_HH_
#define SDF_USD_USD_PARSER_UTILS_HH_

#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/utils/ImplPtr.hh>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
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
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief This class stores the transforms of a schema
    /// This might contain scale, translate or rotation operations
    /// The booleans are used to check if there is a transform defined
    /// in the schema
    /// Rotation is splitted in a vector because this might be defined
    /// as a rotation of 3 angles (ZYX, XYZ, etc).
    class IGNITION_SDFORMAT_USD_VISIBLE UDSTransforms
    {
      /// \brief Default constructor
      public: UDSTransforms();

      /// \brief Translate
      public: ignition::math::Vector3d Translate();

      /// \brief Scale
      public: ignition::math::Vector3d Scale();

      /// \brief Rotation
      public: std::vector<ignition::math::Quaterniond> Rotations();

      /// \brief Set translate
      public: void SetTranslate(const ignition::math::Vector3d &_translate);

      /// \brief Set scale
      public: void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Add rotation
      public: void AddRotation(const ignition::math::Quaterniond &_q);

      /// \brief True if there is a rotation ZYX defined or false otherwise
      bool RotationZYX();

      /// \brief True if there is a rotation XYZ defined or false otherwise
      bool RotationXYZ();

      /// \brief True if there is a rotation (as a quaterion) defined
      /// or false otherwise
      bool Rotation();

      /// \brief Set if there is any rotation ZYX defined
      void SetRotationZYX(bool _rotationZYX);

      /// \brief Set if there is any rotation XYZ defined
      void SetRotationXYZ(bool _rotationXYZ);

      /// \brief Set if there is any rotation defined
      void SetRotation(bool _rotation);

      /// \brief Private data pointer.
      IGN_UTILS_IMPL_PTR(dataPtr)
    };

    /// \brief This function will parse all the parents transforms of a prim
    /// This will stop when the name of the parent is the same as _schemaToStop
    /// \param[in] _prim Initial prim to read the transform
    /// \param[in] _usdData USDData structure to get info about the prim, for
    /// example: metersperunit
    /// \param[out] _tfs A vector with all the transforms
    /// \param[out] _scale The scale of the prims
    /// \param[in] _schemaToStop Name of the prim where the loop will stop
    /// reading transforms
    void GetAllTransforms(
      const pxr::UsdPrim &_prim,
      USDData &_usdData,
      std::vector<ignition::math::Pose3d> &_tfs,
      ignition::math::Vector3d &_scale,
      const std::string &_schemaToStop);

    /// \brief This function get the transform from a prim to the specified
    /// schemaToStop variable
    /// This will stop when the name of the parent is the same as _schemaToStop
    /// \param[in] _prim Initial prim to read the transform
    /// \param[in] _usdData USDData structure to get info about the prim, for
    /// example: metersperunit
    /// \param[out] _pose Pose of the prim
    /// \param[out] _scale The scale of the prim
    /// \param[in] _schemaToStop Name of the prim where the loop will stop
    /// reading transforms
    void IGNITION_SDFORMAT_USD_VISIBLE GetTransform(
      const pxr::UsdPrim &_prim,
      USDData &_usdData,
      ignition::math::Pose3d &_pose,
      ignition::math::Vector3d &_scale,
      const std::string &_schemaToStop);

    /// \brief Read the usd prim transforms. Scale, rotation or transform might
    /// be defined as float or doubles
    /// \param[in] _prim Prim where the transforms are read
    UDSTransforms IGNITION_SDFORMAT_USD_VISIBLE ParseUSDTransform(
      const pxr::UsdPrim &_prim);
}
}
}
#endif
