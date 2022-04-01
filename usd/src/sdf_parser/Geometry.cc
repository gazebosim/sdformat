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

#include "Geometry.hh"

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/math/Vector3.hh>

// TODO(adlarkin) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Box.hh"
#include "sdf/Capsule.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Ellipsoid.hh"
#include "sdf/Geometry.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Sphere.hh"

#include "Material.hh"
#include "../Conversions.hh"
#include "../UsdUtils.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  /// \brief Parse a SDF box geometry into a USD box geometry
  /// \param[in] _geometry The SDF box geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfBoxGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    const auto &sdfBox = _geometry.BoxShape();

    // USD defines a cube (i.e., a box with all dimensions being equal),
    // but not a box. So, we will take a 1x1x1 cube and scale it according
    // to the SDF box's dimensions to achieve varying dimensions
    auto usdCube = pxr::UsdGeomCube::Define(_stage, pxr::SdfPath(_path));
    if (!usdCube)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
            "Unable to define a USD cube geometry at path [" + _path + "]"));
      return errors;
    }

    usdCube.CreateSizeAttr().Set(1.0);
    pxr::GfVec3f endPoint(0.5);
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(-1.0 * endPoint);
    extentBounds.push_back(endPoint);
    usdCube.CreateExtentAttr().Set(extentBounds);

    pxr::UsdGeomXformCommonAPI cubeXformAPI(usdCube);
    cubeXformAPI.SetScale(pxr::GfVec3f(
          sdfBox->Size().X(),
          sdfBox->Size().Y(),
          sdfBox->Size().Z()));

    return errors;
  }

  /// \brief Parse a SDF cylinder geometry into a USD cylinder geometry
  /// \param[in] _geometry The SDF cylinder geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfCylinderGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    const auto &sdfCylinder = _geometry.CylinderShape();

    auto usdCylinder =
      pxr::UsdGeomCylinder::Define(_stage, pxr::SdfPath(_path));
    if (!usdCylinder)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
          "Unable to define a USD cylinder geometry at path [" + _path + "]"));
      return errors;
    }

    usdCylinder.CreateRadiusAttr().Set(sdfCylinder->Radius());
    usdCylinder.CreateHeightAttr().Set(sdfCylinder->Length());
    pxr::GfVec3f endPoint(sdfCylinder->Radius());
    endPoint[2] = sdfCylinder->Length() * 0.5;
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(-1.0 * endPoint);
    extentBounds.push_back(endPoint);
    usdCylinder.CreateExtentAttr().Set(extentBounds);

    return errors;
  }

  /// \brief Parse a SDF sphere geometry into a USD sphere geometry
  /// \param[in] _geometry The SDF sphere geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfSphereGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    const auto &sdfSphere = _geometry.SphereShape();

    auto usdSphere = pxr::UsdGeomSphere::Define(_stage, pxr::SdfPath(_path));
    if (!usdSphere)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
          "Unable to define a USD sphere geometry at path [" + _path + "]"));
      return errors;
    }

    usdSphere.CreateRadiusAttr().Set(sdfSphere->Radius());
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(pxr::GfVec3f(-1.0 * sdfSphere->Radius()));
    extentBounds.push_back(pxr::GfVec3f(sdfSphere->Radius()));
    usdSphere.CreateExtentAttr().Set(extentBounds);

    return errors;
  }

  /// \brief Parse a SDF mesh geometry into a USD mesh geometry
  /// \param[in] _geometry The SDF mesh geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfMeshGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    ignition::common::URI uri(_geometry.MeshShape()->Uri());
    std::string fullName;

    if (uri.Scheme() == "https" || uri.Scheme() == "http")
    {
      fullName =
        ignition::common::findFile(uri.Str());
    }
    else
    {
      fullName =
        ignition::common::findFile(_geometry.MeshShape()->Uri());
    }

    auto ignMesh = ignition::common::MeshManager::Instance()->Load(
        fullName);
    if (!ignMesh)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::MESH_LOAD_FAILURE,
              "Unable to load mesh named [" + fullName + "]"));
      return errors;
    }

    // If a mesh has more than one submesh, only process the submesh who's name
    // is a part of the USD path
    // TODO(adlarkin) figure out if this workaround is actually required. There
    // seemed to be a model that required this workaround, but ahcorde and I
    // cannot remember the particular model that we tested.
    // Most meshes only have one submesh, so this workaround will be ignored for
    // most meshes
    bool subMeshNameInUsdPath = false;
    std::string targetSubMeshName = "";
    if (ignMesh->SubMeshCount() > 1)
    {
      for (unsigned int i = 0; i < ignMesh->SubMeshCount(); ++i)
      {
        auto subMesh = ignMesh->SubMeshByIndex(i).lock();
        if (!subMesh)
        {
          errors.push_back(UsdError(sdf::usd::UsdErrorCode::MESH_LOAD_FAILURE,
                "Unable to get a shared pointer to submesh at index ["
                + std::to_string(i) + "] of parent mesh [" + ignMesh->Name()
                + "]"));
          return errors;
        }

        std::string pathLowerCase = ignition::common::lowercase(_path);
        std::string subMeshLowerCase =
          ignition::common::lowercase(subMesh->Name());

        if (pathLowerCase.find(subMeshLowerCase) != std::string::npos)
        {
          subMeshNameInUsdPath = true;
          targetSubMeshName = subMesh->Name();
          break;
        }
      }
    }

    for (unsigned int i = 0; i < ignMesh->SubMeshCount(); ++i)
    {
      pxr::VtArray<pxr::GfVec3f> meshPoints;
      pxr::VtArray<pxr::GfVec2f> uvs;
      pxr::VtArray<pxr::GfVec3f> normals;
      pxr::VtArray<int> faceVertexIndices;
      pxr::VtArray<int> faceVertexCounts;

      auto subMesh = ignMesh->SubMeshByIndex(i).lock();
      if (!subMesh)
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::MESH_LOAD_FAILURE,
              "Unable to get a shared pointer to submesh at index ["
              + std::to_string(i) + "] of parent mesh [" + ignMesh->Name()
              + "]"));
        return errors;
      }

      if (subMeshNameInUsdPath && (subMesh->Name() != targetSubMeshName))
      {
        continue;
      }

      // copy the submesh's vertices to the usd mesh's "points" array
      for (unsigned int v = 0; v < subMesh->VertexCount(); ++v)
      {
        const auto &vertex = subMesh->Vertex(v);
        meshPoints.push_back(pxr::GfVec3f(vertex.X(), vertex.Y(), vertex.Z()));
      }

      // copy the submesh's indices to the usd mesh's "faceVertexIndices" array
      for (unsigned int j = 0; j < subMesh->IndexCount(); ++j)
        faceVertexIndices.push_back(subMesh->Index(j));

      // copy the submesh's texture coordinates
      for (unsigned int j = 0; j < subMesh->TexCoordCount(); ++j)
      {
        const auto &uv = subMesh->TexCoord(j);
        uvs.push_back(pxr::GfVec2f(uv[0], 1 - uv[1]));
      }

      // copy the submesh's normals
      for (unsigned int j = 0; j < subMesh->NormalCount(); ++j)
      {
        const auto &normal = subMesh->Normal(j);
        normals.push_back(pxr::GfVec3f(normal[0], normal[1], normal[2]));
      }

      // set the usd mesh's "faceVertexCounts" array according to
      // the submesh primitive type
      // TODO(adlarkin) support all primitive types. The computations are more
      // involved for LINESTRIPS, TRIFANS, and TRISTRIPS. I will need to spend
      // some time deriving what the number of faces for these primitive types
      // are, given the number of indices. The "faceVertexCounts" array will
      // also not have the same value for every element in the array for these
      // more complex primitive types (see the TODO note in the for loop below)
      unsigned int verticesPerFace = 0;
      unsigned int numFaces = 0;
      switch (subMesh->SubMeshPrimitiveType())
      {
        case ignition::common::SubMesh::PrimitiveType::POINTS:
          verticesPerFace = 1;
          numFaces = subMesh->IndexCount();
          break;
        case ignition::common::SubMesh::PrimitiveType::LINES:
          verticesPerFace = 2;
          numFaces = subMesh->IndexCount() / 2;
          break;
        case ignition::common::SubMesh::PrimitiveType::TRIANGLES:
          verticesPerFace = 3;
          numFaces = subMesh->IndexCount() / 3;
          break;
        case ignition::common::SubMesh::PrimitiveType::LINESTRIPS:
        case ignition::common::SubMesh::PrimitiveType::TRIFANS:
        case ignition::common::SubMesh::PrimitiveType::TRISTRIPS:
        default:
          errors.push_back(UsdError(
                sdf::usd::UsdErrorCode::INVALID_SUBMESH_PRIMITIVE_TYPE,
                "Submesh " + subMesh->Name() + " has a primitive type that is "
                "not supported."));
          return errors;
      }
      // TODO(adlarkin) update this loop to allow for varying element
      // values in the array (see TODO note above). Right now, the
      // array only allows for all elements to have one value, which in
      // this case is "verticesPerFace"
      for (unsigned int n = 0; n < numFaces; ++n)
        faceVertexCounts.push_back(verticesPerFace);

      std::string primName;
      if (!subMesh->Name().empty())
        primName = _path + "/" + subMesh->Name();
      else
        primName = _path + "/submesh_" + std::to_string(i);

      primName = ignition::common::replaceAll(primName, "-", "_");

      auto usdMesh = pxr::UsdGeomMesh::Define(_stage, pxr::SdfPath(primName));
      if (!usdMesh)
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
            "Unable to define a USD mesh geometry at path [" + primName + "]"));
        return errors;
      }
      usdMesh.CreatePointsAttr().Set(meshPoints);
      usdMesh.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
      usdMesh.CreateFaceVertexCountsAttr().Set(faceVertexCounts);

      auto coordinates = usdMesh.CreatePrimvar(
          pxr::TfToken("st"), pxr::SdfValueTypeNames->Float2Array,
          pxr::UsdGeomTokens->vertex);
      coordinates.Set(uvs);

      usdMesh.CreateNormalsAttr().Set(normals);
      usdMesh.SetNormalsInterpolation(pxr::TfToken("vertex"));

      usdMesh.CreateSubdivisionSchemeAttr(pxr::VtValue(pxr::TfToken("none")));

      const auto &meshMin = ignMesh->Min();
      const auto &meshMax = ignMesh->Max();
      pxr::VtArray<pxr::GfVec3f> extentBounds;
      extentBounds.push_back(
        pxr::GfVec3f(meshMin.X(), meshMin.Y(), meshMin.Z()));
      extentBounds.push_back(
        pxr::GfVec3f(meshMax.X(), meshMax.Y(), meshMax.Z()));
      usdMesh.CreateExtentAttr().Set(extentBounds);

      // TODO(adlarkin) update this call in sdf13 to avoid casting the index to
      // an int:
      // https://github.com/ignitionrobotics/ign-common/pull/319
      int materialIndex = subMesh->MaterialIndex();
      if (materialIndex != -1)
      {
        const auto material = ignMesh->MaterialByIndex(materialIndex).get();
        const sdf::Material materialSdf = sdf::usd::convert(material);
        pxr::SdfPath materialPath;
        UsdErrors materialErrors = ParseSdfMaterial(
          &materialSdf, _stage, materialPath);
        if (!materialErrors.empty())
        {
          errors.push_back(UsdError(
            sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "Unable to convert material [" + std::to_string(materialIndex)
            + "] of submesh named [" + subMesh->Name()
            + "] to a USD material."));
          return errors;
        }

        auto materialPrim = _stage->GetPrimAtPath(materialPath);
        if (!materialPrim)
        {
          errors.push_back(UsdError(
                sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
                "Unable to get material prim at path ["
                + materialPath.GetString()
                + "], but a prim should exist at this path."));
          return errors;
        }

        auto materialUSD = pxr::UsdShadeMaterial(materialPrim);
        if (materialUSD &&
            (materialSdf.Emissive() != ignition::math::Color(0, 0, 0, 1)
             || materialSdf.Specular() != ignition::math::Color(0, 0, 0, 1)
             || materialSdf.PbrMaterial()))
        {
          pxr::UsdShadeMaterialBindingAPI(usdMesh).Bind(materialUSD);
        }
        else if (!materialUSD)
        {
          errors.push_back(UsdError(
              sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
              "The prim at path [" + materialPath.GetString()
              + "] is not a pxr::UsdShadeMaterial object."));
          return errors;
        }
      }

      pxr::UsdGeomXformCommonAPI xform(usdMesh);
      ignition::math::Vector3d scale = _geometry.MeshShape()->Scale();
      xform.SetScale(pxr::GfVec3f{
        static_cast<float>(scale.X()),
        static_cast<float>(scale.Y()),
        static_cast<float>(scale.Z()),
      });
    }

    return errors;
  }

  /// \brief Parse a SDF capsule geometry into a USD capsule geometry
  /// \param[in] _geometry The SDF capsule geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfCapsuleGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    const auto &sdfCapsule = _geometry.CapsuleShape();

    auto usdCapsule = pxr::UsdGeomCapsule::Define(_stage, pxr::SdfPath(_path));
    if (!usdCapsule)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
          "Unable to define a USD capsule geometry at path [" + _path + "]"));
      return errors;
    }

    usdCapsule.CreateRadiusAttr().Set(sdfCapsule->Radius());
    usdCapsule.CreateHeightAttr().Set(sdfCapsule->Length());
    pxr::GfVec3f endPoint(sdfCapsule->Radius());
    endPoint[2] += 0.5 * sdfCapsule->Length();
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(-1.0 * endPoint);
    extentBounds.push_back(endPoint);
    usdCapsule.CreateExtentAttr().Set(extentBounds);

    return errors;
  }

  /// \brief Parse a SDF plane geometry into a USD plane geometry
  /// \param[in] _geometry The SDF plane geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfPlaneGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    const auto &sdfPlane = _geometry.PlaneShape();

    // Currently, there is no USDGeom plane class (it will be added in the
    // future - see
    // https://graphics.pixar.com/usd/release/wp_rigid_body_physics.html#plane-shapes),
    // so the current workaround is to make a large, thin box.
    // To keep things simple for now, a plane will only be parsed if its normal
    // is the unit z vector (0, 0, 1).
    // TODO(adlarkin) support plane normals other than the unit z vector
    // (can update comment above once this functionality is added)
    // TODO(adlarkin) update this to use the pxr::USDGeomPlane class when it's
    // added
    if (sdfPlane->Normal() != ignition::math::Vector3d::UnitZ)
    {
      errors.push_back(UsdError(
            sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Only planes with a unit z vector normal are supported.")));
      return errors;
    }

    sdf::Box box;
    box.SetSize(ignition::math::Vector3d(
          sdfPlane->Size().X(), sdfPlane->Size().Y(), usd::kPlaneThickness));

    sdf::Geometry planeBoxGeometry;
    planeBoxGeometry.SetType(sdf::GeometryType::BOX);
    planeBoxGeometry.SetBoxShape(box);

    return ParseSdfBoxGeometry(planeBoxGeometry, _stage, _path);
  }

  /// \brief Parse a SDF ellipsoid geometry into a USD ellipsoid geometry
  /// \param[in] _geometry The SDF ellipsoid geometry
  /// \param[in] _stage The stage that will contain the parsed USD equivalent
  /// of _geometry
  /// \param[in] _path Where the parsed USD equivalent of _geometry should exist
  /// in _stage
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when creating the USD equivalent of _geometry
  UsdErrors ParseSdfEllipsoid(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    const auto sdfEllipsoid = _geometry.EllipsoidShape();

    auto usdEllipsoid = pxr::UsdGeomSphere::Define(_stage, pxr::SdfPath(_path));
    if (!usdEllipsoid)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
          "Unable to define a USD ellipsoid geometry at path [" + _path + "]"));
      return errors;
    }

    usdEllipsoid.CreateRadiusAttr().Set(1.0);
    pxr::UsdGeomXformCommonAPI xform(usdEllipsoid);
    xform.SetScale(pxr::GfVec3f{
      static_cast<float>(sdfEllipsoid->Radii().X()),
      static_cast<float>(sdfEllipsoid->Radii().Y()),
      static_cast<float>(sdfEllipsoid->Radii().Z()),
    });
    // extents is the bounds before any transformation
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(pxr::GfVec3f{-1.0f});
    extentBounds.push_back(pxr::GfVec3f{1.0f});
    usdEllipsoid.CreateExtentAttr().Set(extentBounds);

    return errors;
  }

  UsdErrors ParseSdfGeometry(const sdf::Geometry &_geometry,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;
    switch (_geometry.Type())
    {
      case sdf::GeometryType::BOX:
        errors = ParseSdfBoxGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::CYLINDER:
        errors = ParseSdfCylinderGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::SPHERE:
        errors = ParseSdfSphereGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::MESH:
        errors = ParseSdfMeshGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::CAPSULE:
        errors = ParseSdfCapsuleGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::PLANE:
        errors = ParseSdfPlaneGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::ELLIPSOID:
        errors = ParseSdfEllipsoid(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::HEIGHTMAP:
      default:
        errors.push_back(UsdError(
              sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
                "Geometry type is either invalid or not supported")));
    }

    // Set the collision for this geometry.
    // In SDF, the collisions of a link are defined separately from
    // the link's visual geometries (in case a user wants to define a simpler
    // collision, for example). In USD, the collision can be attached to the
    // geometry so that the collision shape is the geometry shape.
    // The current approach that's taken here (attaching a collision to the
    // geometry) assumes that for every visual in a link, the visual should have
    // a collision that matches its geometry. This approach currently ignores
    // collisions defined in a link since it instead creates collisions for a
    // link that match a link's visual geometries.
    // TODO(adlarkin) support the option of a different collision shape (i.e.,
    // don't ignore collisions defined in a link),
    // especially for meshes - here's more information about how to do this:
    // https://graphics.pixar.com/usd/release/wp_rigid_body_physics.html?highlight=collision#turning-meshes-into-shapes
    if (errors.empty())
    {
      auto geomPrim = _stage->GetPrimAtPath(pxr::SdfPath(_path));
      if (!geomPrim)
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
          "Internal error: unable to get prim at path ["
          + _path + "], but a geom prim should exist at this path"));
        return errors;
      }

      if (!pxr::UsdPhysicsCollisionAPI::Apply(geomPrim))
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
          "Internal error: unable to apply a collision to the prim at path ["
          + _path + "]"));
        return errors;
      }
    }

    return errors;
  }
}
}
}
