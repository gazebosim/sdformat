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

#include "sdf_usd_parser/geometry.hh"

#include <iostream>
#include <string>

#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/SubMesh.hh>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include "sdf/Box.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Mesh.hh"
#include "sdf/Sphere.hh"
#include "sdf/Capsule.hh"

namespace usd
{
  bool ParseSdfBoxGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const auto &sdfBox = _geometry.BoxShape();

    // USD defines a cube (i.e., a box with all dimensions being equal),
    // but not a box. So, we will take a 1x1x1 cube and scale it according
    // to the SDF box's dimensions to achieve varying dimensions
    auto usdCube = pxr::UsdGeomCube::Define(_stage, pxr::SdfPath(_path));
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

    return true;
  }

  bool ParseSdfCylinderGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const auto &sdfCylinder = _geometry.CylinderShape();

    auto usdCylinder = pxr::UsdGeomCylinder::Define(_stage, pxr::SdfPath(_path));
    usdCylinder.CreateRadiusAttr().Set(sdfCylinder->Radius());
    usdCylinder.CreateHeightAttr().Set(sdfCylinder->Length());
    pxr::GfVec3f endPoint(sdfCylinder->Radius());
    endPoint[2] = sdfCylinder->Length() * 0.5;
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(-1.0 * endPoint);
    extentBounds.push_back(endPoint);
    usdCylinder.CreateExtentAttr().Set(extentBounds);

    return true;
  }

  bool ParseSdfSphereGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const auto &sdfSphere = _geometry.SphereShape();

    auto usdSphere = pxr::UsdGeomSphere::Define(_stage, pxr::SdfPath(_path));
    usdSphere.CreateRadiusAttr().Set(sdfSphere->Radius());
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(pxr::GfVec3f(-1.0 * sdfSphere->Radius()));
    extentBounds.push_back(pxr::GfVec3f(sdfSphere->Radius()));
    usdSphere.CreateExtentAttr().Set(extentBounds);

    return true;
  }

  bool ParseSdfMeshGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    auto ignMesh = ignition::common::MeshManager::Instance()->Load(
        _geometry.MeshShape()->Uri());

    pxr::VtArray<pxr::GfVec3f> meshPoints;
    pxr::VtArray<pxr::GfVec2f> uvs;
    pxr::VtArray<pxr::GfVec3f> normals;
    pxr::VtArray<int> faceVertexIndices;
    pxr::VtArray<int> faceVertexCounts;
    for (unsigned int i = 0; i < ignMesh->SubMeshCount(); ++i)
    {
      auto subMesh = ignMesh->SubMeshByIndex(i).lock();
      if (!subMesh)
      {
        std::cerr << "Unable to get a shared pointer to submesh at index ["
                  << i << "] of parent mesh [" << ignMesh->Name() << "]\n";
        return false;
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

      // copy the submesh's texture coordinate to the usd mesh's
      // "faceVertexIndices" array
      for (unsigned int j = 0; j < subMesh->TexCoordCount(); ++j)
      {
        const auto &uv = subMesh->TexCoord(j);
        uvs.push_back(pxr::GfVec2f(uv[0], 1- uv[1]));
      }

      // copy the submesh's normals to the usd mesh's "faceVertexIndices" array
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
          std::cerr << "Submesh " << subMesh->Name()
                    << " has a primitive type that is not supported.\n";
          return false;
      }
      // TODO(adlarkin) update this loop to allow for varying element
      // values in the array (see TODO note above). Right now, the
      // array only allows for all elements to have one value, which in
      // this case is "verticesPerFace"
      for (unsigned int n = 0; n < numFaces; ++n)
        faceVertexCounts.push_back(verticesPerFace);
    }
    auto usdMesh = pxr::UsdGeomMesh::Define(_stage, pxr::SdfPath(_path));
    usdMesh.CreatePointsAttr().Set(meshPoints);
    usdMesh.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
    usdMesh.CreateFaceVertexCountsAttr().Set(faceVertexCounts);

    auto coordinates = usdMesh.CreatePrimvar(
        pxr::TfToken("st"), pxr::SdfValueTypeNames->Float2Array,
        pxr::UsdGeomTokens->vertex);

    coordinates.Set(uvs);

    usdMesh.CreateNormalsAttr().Set(normals);

    const auto &meshMin = ignMesh->Min();
    const auto &meshMax = ignMesh->Max();
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(pxr::GfVec3f(meshMin.X(), meshMin.Y(), meshMin.Z()));
    extentBounds.push_back(pxr::GfVec3f(meshMax.X(), meshMax.Y(), meshMax.Z()));
    usdMesh.CreateExtentAttr().Set(extentBounds);

    return true;
  }

  bool ParseSdfCapsuleGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const auto &sdfCapsule = _geometry.CapsuleShape();

    auto usdCapsule = pxr::UsdGeomCapsule::Define(_stage, pxr::SdfPath(_path));
    usdCapsule.CreateRadiusAttr().Set(sdfCapsule->Radius());
    usdCapsule.CreateHeightAttr().Set(sdfCapsule->Length());
    pxr::GfVec3f endPoint(sdfCapsule->Radius());
    endPoint[2] += 0.5 * sdfCapsule->Length();
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(-1.0 * endPoint);
    extentBounds.push_back(endPoint);
    usdCapsule.CreateExtentAttr().Set(extentBounds);

    return true;
  }

  bool ParseSdfGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    bool typeParsed = false;
    switch (_geometry.Type())
    {
      case sdf::GeometryType::BOX:
        typeParsed = ParseSdfBoxGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::CYLINDER:
        typeParsed = ParseSdfCylinderGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::SPHERE:
        typeParsed = ParseSdfSphereGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::MESH:
        typeParsed = ParseSdfMeshGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::CAPSULE:
        typeParsed = ParseSdfCapsuleGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::PLANE:
      case sdf::GeometryType::ELLIPSOID:
      case sdf::GeometryType::HEIGHTMAP:
      default:
        std::cerr << "Geometry type is either invalid or not supported\n";
    }

    return typeParsed;
  }
}
