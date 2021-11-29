#include "USDStage.hh"

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>

#include "usd_parser/utils.hh"

namespace usd {

  USDStage::USDStage(const std::string &_refFileName)
  {
    auto referencee = pxr::UsdStage::Open(_refFileName);
    if (!referencee)
    {
      return;
    }
    referencee->GetMetadata<double>(
      pxr::TfToken("metersPerUnit"), &this->_metersPerUnit);

    if (referencee->HasAuthoredMetadata(pxr::UsdGeomTokens->upAxis))
    {
      pxr::TfToken axis;
      referencee->GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
      this->_upAxis = axis.GetText();
    }

    this->_referenceName = referencee->GetDefaultPrim().GetName().GetText();
    this->_filename = _refFileName;
    auto range = pxr::UsdPrimRange::Stage(referencee);

    for (auto const &_prim : range)
    {
      if (_prim.IsA<pxr::UsdShadeMaterial>())
      {
        continue;
      }
      if (_prim.IsA<pxr::UsdShadeShader>())
      {
        continue;
      }
      // std::string primName = pxr::TfStringify(_prim.GetPath());
      // std::string usdPath = removeSubStr(primName, "/" + this->_referenceName);
      // usdPath = removeSubStr(usdPath, "/Looks");
      // if (usdPath[0] == '/')
      // {
      //   usdPath.erase(0, 1);
      // }
      // if (!usdPath.empty())
      // {
        this->_paths.insert(_prim.GetPath().GetName());
      // }
    }
  }
}
