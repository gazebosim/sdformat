/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef USD_MODEL_UTILS_HH
#define USD_MODEL_UTILS_HH

#include <tuple>

#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/rotation.h>

#include "pxr/usd/usdGeom/gprim.h"

#include "sdf/Material.hh"

namespace usd{

  sdf::Material ParseMaterial(const pxr::UsdPrim &_prim);

  template<typename T1, typename T2, typename T3>
  std::tuple<T1, T2, T3, bool, bool, bool> ParseTransform(
    const pxr::UsdPrim &_prim)
  {
    auto variant_geom = pxr::UsdGeomGprim(_prim);

    auto transforms = variant_geom.GetXformOpOrderAttr();

    T1 scale(1, 1, 1);
    T2 translate(0, 0, 0);
    T3 rotation_quad(1, 0, 0, 0);

    bool isScale = false;
    bool isTranslate = false;
    bool isRotation = false;

    pxr::VtTokenArray xformOpOrder;
    transforms.Get(&xformOpOrder);
    for (auto & op: xformOpOrder)
    {
      std::cerr << "xformOpOrder " << op << '\n';
      std::string s = op;
      if (op == "xformOp:scale")
      {
        _prim.GetAttribute(pxr::TfToken("xformOp:scale")).Get(&scale);
        std::cerr << "scale "<< scale << '\n';
        isScale = true;
      }

      if (op == "xformOp:translate")
      {
        _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
        std::cerr << "translate " << translate << '\n';
        isTranslate = true;
      }
      if (op == "xformOp:orient")
      {
        _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
        std::cerr << "rotation_quad " << rotation_quad << '\n';
        isRotation = true;
      }

      if (op == "xformOp:transform")
      {
        pxr::GfMatrix4d transform;
        _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
        pxr::GfVec3d translateMatrix = transform.ExtractTranslation();
        pxr::GfQuatd rotation_quadMatrix = transform.ExtractRotationQuat();
        translate[0] = translateMatrix[0];
        translate[1] = translateMatrix[1];
        translate[2] = translateMatrix[2];
        rotation_quad.SetImaginary(
          rotation_quadMatrix.GetImaginary()[0],
          rotation_quadMatrix.GetImaginary()[1],
          rotation_quadMatrix.GetImaginary()[2]);
        rotation_quad.SetReal(rotation_quadMatrix.GetReal());
        isTranslate = true;
        isRotation = true;
        std::cerr << "translate " << translateMatrix << '\n';
        std::cerr << "rotation_quad " << rotation_quadMatrix << '\n';
      }
    }

    return std::make_tuple(scale, translate, rotation_quad, isScale, isTranslate, isRotation);
  }
}

#endif
