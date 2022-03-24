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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/utils/ImplPtr.hh>

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
    /// Rotation is splitted in a vector because this might be defined
    /// as a rotation of 3 angles (ZYX, XYZ, etc).
    class IGNITION_SDFORMAT_USD_VISIBLE UDSTransforms
    {
      /// \brief Default constructor
      public: UDSTransforms();

      /// \brief Translate
      /// \return A 3D vector with the translation
      public: const ignition::math::Vector3d Translate() const;

      /// \brief Scale
      /// \return A 3D vector with the scale
      public: const ignition::math::Vector3d Scale() const;

      /// \brief Rotation
      /// \return Return a vector with all the rotations
      /// If RotationXYZ or RotationZYY is true, this method will return a
      /// vector of 3 quaternions, Rotation<axis1><axis2><axis3> with the first
      /// quaternion being rotation <axis1>, the second being rotation about
      /// <axis2>, and the third being rotation about <axis3>
      public: const std::vector<ignition::math::Quaterniond> Rotations() const;

      /// \brief Set translate
      /// \param[in] _translate Translate to set
      public: void Translate(const ignition::math::Vector3d &_translate);

      /// \brief Set scale
      /// \param[in] _scale Scale to set
      public: void Scale(const ignition::math::Vector3d &_scale);

      /// \brief Add rotation
      /// \param[in] _q Quaternion to add to the list of rotations
      public: void AddRotation(const ignition::math::Quaterniond &_q);

      /// \brief True if there is a rotation ZYX defined or false otherwise
      public: bool RotationZYX() const;

      /// \brief True if there is a rotation XYZ defined or false otherwise
      public: bool RotationXYZ() const;

      /// \brief True if there is a rotation (as a quaternion) defined
      /// or false otherwise
      public: bool Rotation() const;

      /// \brief Set if there is any rotation ZYX defined
      /// RotationZYX is used to determine the order of stored rotations
      /// If RotationZYX is true, then Rotation should be True too
      /// If Rotation is false, then RotationZYX cannot be true
      /// \param[in] _rotationZYX If the rotation is ZYX (true) or not (false)
      public: void RotationZYX(bool _rotationZYX);

      /// \brief Set if there is any rotation XYZ defined
      /// RotationXYZ is used to determine the order of stored rotations
      /// If RotationXYZ is true, then Rotation should be True too
      /// If Rotation is false, then RotationXYZ cannot be true
      /// \param[in] _rotationXYZ If the rotation is XYZ (true) or not (false)
      public: void RotationXYZ(bool _rotationXYZ);

      /// \brief Set if there is any rotation defined
      /// \param[in] _rotation If there is any rotation defined (true)
      /// or not (false)
      public: void Rotation(bool _rotation);

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
      USDData &_usdData,
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
