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

#include "USDLinks.hh"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/subset.h>
#include "pxr/usd/usdPhysics/collisionAPI.h"
#include "pxr/usd/usdPhysics/massAPI.h"
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/ColladaExporter.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Material.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/Util.hh>

#include <ignition/math/Inertial.hh>

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Sphere.hh"
#include "sdf/Visual.hh"

#include "polygon_helper.hh"

#include "sdf/usd/usd_parser/USDTransforms.hh"
#include "sdf/usd/UsdError.hh"
#include "../Conversions.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
//////////////////////////////////////////////////
void GetInertial(
  const pxr::UsdPrim &_prim,
  ignition::math::Inertiald &_inertial)
{
  float mass;
  pxr::GfVec3f centerOfMass;
  pxr::GfVec3f diagonalInertia;
  pxr::GfQuatf principalAxes;

  ignition::math::MassMatrix3d massMatrix;

  if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
  {
    if (pxr::UsdAttribute massAttribute =
      _prim.GetAttribute(pxr::TfToken("physics:mass")))
    {
      massAttribute.Get(&mass);

      if (pxr::UsdAttribute centerOfMassAttribute =
        _prim.GetAttribute(pxr::TfToken("physics:centerOfMass")))
      {
        centerOfMassAttribute.Get(&centerOfMass);
      }

      if (pxr::UsdAttribute diagonalInertiaAttribute =
        _prim.GetAttribute(pxr::TfToken("physics:diagonalInertia")))
      {
        diagonalInertiaAttribute.Get(&diagonalInertia);
      }

      if (pxr::UsdAttribute principalAxesAttribute =
        _prim.GetAttribute(pxr::TfToken("physics:principalAxes")))
      {
        principalAxesAttribute.Get(&principalAxes);
      }

      // Added a diagonal inertia to avoid crash with the physics engine
      if (diagonalInertia == pxr::GfVec3f(0, 0, 0))
      {
        diagonalInertia = pxr::GfVec3f(0.0001, 0.0001, 0.0001);
      }

      // Added a mass to avoid crash with the physics engine
      if (mass < 0.0001)
      {
        mass = 0.1;
      }

      massMatrix.SetMass(mass);

      // TODO(ahcorde) Figure out how to use PrincipalMoments and
      // PrincipalAxesOffset: see
      // https://github.com/ignitionrobotics/sdformat/pull/902#discussion_r840905534
      massMatrix.SetDiagonalMoments(
        ignition::math::Vector3d(
          diagonalInertia[0],
          diagonalInertia[1],
          diagonalInertia[2]));

      _inertial.SetPose(ignition::math::Pose3d(
          ignition::math::Vector3d(
            centerOfMass[0], centerOfMass[1], centerOfMass[2]),
          ignition::math::Quaterniond(1.0, 0, 0, 0)));

      _inertial.SetMassMatrix(massMatrix);
    }
  }
  else
  {
    auto parent = _prim.GetParent();
    if (parent)
    {
      GetInertial(parent, _inertial);
    }
  }
}

//////////////////////////////////////////////////
/// \brief Create the directory based on the USD path
/// \param[in] _primPath Reference to the USD prim
/// \return A string with the path
std::string directoryFromUSDPath(const std::string &_primPath)
{
  std::vector<std::string> tokensChild =
    ignition::common::split(_primPath, "/");
  std::string directoryMesh;
  if (tokensChild.size() > 1)
  {
    directoryMesh = tokensChild[0];
    for (unsigned int i = 1; i < tokensChild.size() - 1; ++i)
    {
        directoryMesh = ignition::common::joinPaths(
          directoryMesh, tokensChild[i+1]);
    }
  }
  return directoryMesh;
}

//////////////////////////////////////////////////
/// \brief Get the material name of the prim
/// \param[in] _prim Reference to the USD prim
/// \return A string with the material name
std::string ParseMaterialName(const pxr::UsdPrim &_prim)
{
  // Materials
  std::string nameMaterial;

  auto bindMaterial = pxr::UsdShadeMaterialBindingAPI(_prim);
  pxr::UsdRelationship directBindingRel =
    bindMaterial.GetDirectBindingRel();

  pxr::SdfPathVector paths;
  directBindingRel.GetTargets(&paths);
  for (const auto & p : paths)
  {
    std::vector<std::string> tokensMaterial =
      ignition::common::split(pxr::TfStringify(p), "/");

    if(tokensMaterial.size() > 0)
    {
      nameMaterial = tokensMaterial[tokensMaterial.size() - 1];
    }
  }
  return nameMaterial;
}

//////////////////////////////////////////////////
/// \brief Parse USD mesh subsets
/// \param[in] _prim Reference to the USD prim
/// \param[in] _link Current link
/// \param[in] _subMesh submesh to add the subsets
/// \param[in] _meshGeom Mesh geom
/// \param[inout] _scale scale mesh
/// \param[in] _usdData metadata of the USD file
/// \return Number of mesh subsets
int ParseMeshSubGeom(const pxr::UsdPrim &_prim,
  sdf::Link *_link,
  ignition::common::SubMesh &_subMesh,
  sdf::Mesh &_meshGeom,
  ignition::math::Vector3d &_scale,
  const USDData &_usdData)
{
  int numSubMeshes = 0;

  for (const auto & child : _prim.GetChildren())
  {
    if (child.IsA<pxr::UsdGeomSubset>())
    {
      sdf::Visual visSubset;
      std::string nameMaterial = ParseMaterialName(child);
      auto it = _usdData.Materials().find(nameMaterial);
      if (it != _usdData.Materials().end())
      {
        visSubset.SetMaterial(it->second);
      }

      ignition::common::Mesh meshSubset;

      numSubMeshes++;

      ignition::common::SubMesh subMeshSubset;
      subMeshSubset.SetPrimitiveType(ignition::common::SubMesh::TRISTRIPS);
      subMeshSubset.SetName("subgeommesh");

      if (it != _usdData.Materials().end())
      {
        std::shared_ptr<ignition::common::Material> matCommon =
          std::make_shared<ignition::common::Material>();
        convert(it->second, *matCommon.get());
        meshSubset.AddMaterial(matCommon);
        subMeshSubset.SetMaterialIndex(meshSubset.MaterialCount() - 1);
      }
      pxr::VtIntArray faceVertexIndices;
      child.GetAttribute(pxr::TfToken("indices")).Get(&faceVertexIndices);

      for (unsigned int i = 0; i < faceVertexIndices.size(); ++i)
      {
        subMeshSubset.AddIndex(_subMesh.Index(faceVertexIndices[i] * 3));
        subMeshSubset.AddIndex(_subMesh.Index(faceVertexIndices[i] * 3 + 1));
        subMeshSubset.AddIndex(_subMesh.Index(faceVertexIndices[i] * 3 + 2));
      }

      for (unsigned int i = 0; i < _subMesh.VertexCount(); ++i)
      {
        subMeshSubset.AddVertex(_subMesh.Vertex(i));
      }
      for (unsigned int i = 0; i < _subMesh.NormalCount(); ++i)
      {
        subMeshSubset.AddNormal(_subMesh.Normal(i));
      }
      for (unsigned int i = 0; i < _subMesh.TexCoordCount(); ++i)
      {
        subMeshSubset.AddTexCoord(_subMesh.TexCoord(i));
      }

      meshSubset.AddSubMesh(subMeshSubset);
      sdf::Mesh meshGeomSubset;
      sdf::Geometry geomSubset;
      geomSubset.SetType(sdf::GeometryType::MESH);

      std::string childPathName = pxr::TfStringify(child.GetPath());
      std::string directoryMesh = directoryFromUSDPath(childPathName) +
        childPathName;

      if (ignition::common::createDirectories(directoryMesh))
      {
        directoryMesh = ignition::common::joinPaths(directoryMesh,
          ignition::common::basename(directoryMesh));

        // Export with extension
        ignition::common::ColladaExporter exporter;
        exporter.Export(&meshSubset, directoryMesh, false);
      }
      _meshGeom.SetFilePath(directoryMesh + ".dae");
      _meshGeom.SetUri(directoryMesh + ".dae");

      geomSubset.SetMeshShape(_meshGeom);
      visSubset.SetName("mesh_subset");
      visSubset.SetGeom(geomSubset);

      ignition::math::Pose3d pose;
      ignition::math::Vector3d scale(1, 1, 1);
      std::string linkName = pxr::TfStringify(_prim.GetPath());
      auto found = linkName.find(_link->Name());
      if (found != std::string::npos)
      {
        linkName = linkName.substr(0, found + _link->Name().size());
      }
      GetTransform(child, _usdData, pose, scale, linkName);
      _scale *= scale;
      visSubset.SetRawPose(pose);
      _link->AddVisual(visSubset);
    }
  }
  return numSubMeshes;
}

//////////////////////////////////////////////////
/// \brief Parse USD mesh
/// \param[in] _prim Reference to the USD prim
/// \param[in] _link Current link
/// \param[in] _vis sdf visual
/// \param[in] _geom sdf geom
/// \param[in] _scale scale mesh
/// \param[in] _usdData metadata of the USD file
/// \param[out] _pose The pose of the parsed mesh
/// \return UsdErrors, which is a list of UsdError objects. An empty list means
/// that no errors occurred when parsing the USD mesh
UsdErrors ParseMesh(
  const pxr::UsdPrim &_prim,
  sdf::Link *_link,
  sdf::Visual &_vis,
  sdf::Geometry &_geom,
  ignition::math::Vector3d &_scale,
  const USDData &_usdData,
  ignition::math::Pose3d &_pose)
{
  UsdErrors errors;

  const std::pair<std::string, std::shared_ptr<USDStage>> data =
    _usdData.FindStage(_prim.GetPath().GetName());

  double metersPerUnit = data.second->MetersPerUnit();

  ignition::common::Mesh mesh;
  ignition::common::SubMesh subMesh;
  subMesh.SetPrimitiveType(ignition::common::SubMesh::TRISTRIPS);
  pxr::VtIntArray faceVertexIndices;
  pxr::VtIntArray faceVertexCounts;
  pxr::VtArray<pxr::GfVec3f> normals;
  pxr::VtArray<pxr::GfVec3f> points;
  pxr::VtArray<pxr::GfVec2f> textCoords;
  _prim.GetAttribute(pxr::TfToken("faceVertexCounts")).Get(&faceVertexCounts);
  _prim.GetAttribute(pxr::TfToken("faceVertexIndices")).Get(&faceVertexIndices);
  _prim.GetAttribute(pxr::TfToken("normals")).Get(&normals);
  _prim.GetAttribute(pxr::TfToken("points")).Get(&points);
  _prim.GetAttribute(pxr::TfToken("primvars:st")).Get(&textCoords);

  if (textCoords.size() == 0)
  {
    _prim.GetAttribute(pxr::TfToken("primvars:st_0")).Get(&textCoords);
  }

  std::vector<unsigned int> indexes;
  errors = PolygonToTriangles(
    faceVertexIndices, faceVertexCounts, indexes);
  if (!errors.empty())
  {
    errors.emplace_back(UsdErrorCode::USD_TO_SDF_POLYGON_PARSING_ERROR,
      "Unable to parse polygon in path [" + pxr::TfStringify(_prim.GetPath())
      + "]");
    return errors;
  }

  for (unsigned int i = 0; i < indexes.size(); ++i)
  {
    subMesh.AddIndex(indexes[i]);
  }

  for (const auto & textCoord : textCoords)
  {
    subMesh.AddTexCoord(textCoord[0], (1 - textCoord[1]));
  }

  for (const auto & point : points)
  {
    ignition::math::Vector3d v =
      ignition::math::Vector3d(point[0], point[1], point[2]) * metersPerUnit;
    subMesh.AddVertex(v);
  }

  for (const auto & normal : normals)
  {
    subMesh.AddNormal(normal[0], normal[1], normal[2]);
  }

  sdf::Mesh meshGeom;
  _geom.SetType(sdf::GeometryType::MESH);

  ignition::math::Pose3d pose;
  ignition::math::Vector3d scale(1, 1, 1);
  std::string linkName = pxr::TfStringify(_prim.GetPath());
  auto found = linkName.find(_link->Name());
  if (found != std::string::npos)
  {
    linkName = linkName.substr(0, found + _link->Name().size());
  }

  size_t nSlash = std::count(linkName.begin(), linkName.end(), '/');
  if (nSlash == 1)
  {
    GetTransform(_prim, _usdData, pose, scale, "/");
  }
  else
  {
    GetTransform(_prim, _usdData, pose, scale, linkName);
  }

  _pose = pose;

  meshGeom.SetScale(scale * _scale);

  std::string primName = pxr::TfStringify(_prim.GetPath());

  std::string directoryMesh = directoryFromUSDPath(primName) + primName;

  meshGeom.SetFilePath(
    ignition::common::joinPaths(
      directoryMesh, ignition::common::basename(directoryMesh)) + ".dae");

  meshGeom.SetUri(meshGeom.FilePath());

  int numSubMeshes = ParseMeshSubGeom(
    _prim, _link, subMesh, meshGeom, _scale, _usdData);

  _geom.SetMeshShape(meshGeom);

  if (numSubMeshes == 0)
  {
    std::string nameMaterial = ParseMaterialName(_prim);
    auto it = _usdData.Materials().find(nameMaterial);
    if (it != _usdData.Materials().end())
    {
      _vis.SetMaterial(it->second);
      std::shared_ptr<ignition::common::Material> matCommon =
        std::make_shared<ignition::common::Material>();
      convert(it->second, *matCommon.get());
      mesh.AddMaterial(matCommon);
      subMesh.SetMaterialIndex(mesh.MaterialCount() - 1);
    }

    std::string primNameStr = _prim.GetPath().GetName();
    _vis.SetName(primNameStr + "_visual");
    _vis.SetRawPose(pose);
    _vis.SetGeom(_geom);

    _link->AddVisual(_vis);

    mesh.AddSubMesh(subMesh);

    if (ignition::common::createDirectories(directoryMesh))
    {
      directoryMesh = ignition::common::joinPaths(
        directoryMesh, ignition::common::basename(directoryMesh));
      // Export with extension
      ignition::common::ColladaExporter exporter;
      exporter.Export(&mesh, directoryMesh, false);
    }
  }

  return errors;
}

//////////////////////////////////////////////////
/// \brief Parse USD cube
/// \param[in] _prim Reference to the USD prim
/// \param[in] _geom sdf geom
/// \param[in] _scale scale mesh
/// \param[in] _metersPerUnit meter per unit of the stage
void ParseCube(const pxr::UsdPrim &_prim,
  sdf::Geometry &_geom,
  const ignition::math::Vector3d &_scale,
  double _metersPerUnit)
{
  double size;
  auto variant_cube = pxr::UsdGeomCube(_prim);
  variant_cube.GetSizeAttr().Get(&size);

  size = size * _metersPerUnit;

  sdf::Box box;
  _geom.SetType(sdf::GeometryType::BOX);
  box.SetSize(ignition::math::Vector3d(
    size * _scale.X(),
    size * _scale.Y(),
    size * _scale.Z()));

  _geom.SetBoxShape(box);
}

//////////////////////////////////////////////////
/// \brief Parse USD sphere
/// \param[in] _prim Reference to the USD prim
/// \param[in] _geom sdf geom
/// \param[in] _scale scale mesh
/// \param[in] _metersPerUnit meter per unit of the stage
void ParseSphere(const pxr::UsdPrim &_prim,
  sdf::Geometry &_geom,
  const ignition::math::Vector3d &_scale,
  double _metersPerUnit)
{
  double radius;
  auto variant_sphere = pxr::UsdGeomSphere(_prim);
  variant_sphere.GetRadiusAttr().Get(&radius);

  sdf::Sphere s;
  _geom.SetType(sdf::GeometryType::SPHERE);
  s.SetRadius(radius * _metersPerUnit * _scale.X());
  _geom.SetSphereShape(s);
}

//////////////////////////////////////////////////
/// \brief Parse USD cylinder
/// \param[in] _prim Reference to the USD prim
/// \param[in] _geom sdf geom
/// \param[in] _scale scale mesh
/// \param[in] _metersPerUnit meter per unit of the stage
void ParseCylinder(
  const pxr::UsdPrim &_prim,
  sdf::Geometry &_geom,
  const ignition::math::Vector3d &_scale,
  double _metersPerUnit)
{
  auto variant_cylinder = pxr::UsdGeomCylinder(_prim);
  double radius;
  double height;
  variant_cylinder.GetRadiusAttr().Get(&radius);
  variant_cylinder.GetHeightAttr().Get(&height);

  sdf::Cylinder c;
  _geom.SetType(sdf::GeometryType::CYLINDER);

  c.SetRadius(radius * _metersPerUnit * _scale.X());
  c.SetLength(height * _metersPerUnit * _scale.Z());

  _geom.SetCylinderShape(c);
}

//////////////////////////////////////////////////
UsdErrors ParseUSDLinks(
  const pxr::UsdPrim &_prim,
  const std::string &_nameLink,
  std::optional<sdf::Link> &_link,
  const USDData &_usdData,
  ignition::math::Vector3d &_scale)
{
  UsdErrors errors;
  const std::string primNameStr = _prim.GetPath().GetName();
  const std::string primPathStr = pxr::TfStringify(_prim.GetPath());
  const std::string primType = _prim.GetPrimTypeInfo().GetTypeName().GetText();

  const std::pair<std::string, std::shared_ptr<USDStage>> data =
    _usdData.FindStage(primNameStr);

  // Is this a new link ?
  if (!_link)
  {
    _link = sdf::Link();
    _link->SetName(ignition::common::basename(_nameLink));

    // USD define visual inside other visuals or links
    // This loop allow to find the link for a specific visual
    // For example
    // This visual /ur10_long_suction/shoulder_link/cylinder
    // correponds to the link /ur10_long_suction/wrist_3_link
    // Then we can find the right transformations
    std::string originalPrimName = pxr::TfStringify(_prim.GetPath());
    size_t pos = std::string::npos;
    std::string nameOfLink;
    if ((pos = originalPrimName.find(_link->Name()))!= std::string::npos)
    {
      nameOfLink = originalPrimName.erase(
        pos + _link->Name().length(),
        originalPrimName.length() - (pos + _link->Name().length()));
    }
    pxr::UsdPrim tmpPrim = _prim;
    if (!nameOfLink.empty())
    {
      while (tmpPrim)
      {
        if (pxr::TfStringify(tmpPrim.GetPath()) == nameOfLink)
        {
          break;
        }
        tmpPrim = tmpPrim.GetParent();
      }
    }

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale(1, 1, 1);
    GetTransform(tmpPrim, _usdData, pose, scale, "");
    // This is a special case when a geometry is defined in the higher level
    // of the path. we should only set the position if the path at least has
    // more than 1 level.
    // TODO(ahcorde) Review this code and improve this logic
    size_t nSlash = std::count(_nameLink.begin(), _nameLink.end(), '/');
    if (nSlash > 1)
      _link->SetRawPose(pose);
    _scale *= scale;
  }

  // If the schema is a rigid body use this name instead.
  if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
  {
    _link->SetName(ignition::common::basename(primPathStr));
  }

  ignition::math::Inertiald noneInertial = {{1.0,
            ignition::math::Vector3d::One, ignition::math::Vector3d::Zero},
            ignition::math::Pose3d::Zero};
  const auto inertial = _link->Inertial();
  if (inertial == noneInertial)
  {
    ignition::math::Inertiald newInertial;
    GetInertial(_prim, newInertial);
    _link->SetInertial(newInertial);
  }

  sdf::Geometry geom;
  if (_prim.IsA<pxr::UsdGeomSphere>() ||
      _prim.IsA<pxr::UsdGeomCylinder>() ||
      _prim.IsA<pxr::UsdGeomCube>() ||
      _prim.IsA<pxr::UsdGeomMesh>() ||
      primType == "Plane")
  {
    sdf::Visual vis;

    auto variant_geom = pxr::UsdGeomGprim(_prim);

    bool collisionEnabled = false;
    if (_prim.HasAPI<pxr::UsdPhysicsCollisionAPI>())
    {
      _prim.GetAttribute(
        pxr::TfToken("physics:collisionEnabled")).Get(&collisionEnabled);
    }

    pxr::TfToken kindOfSchema;
    if(!pxr::UsdModelAPI(_prim).GetKind(&kindOfSchema))
    {
      auto parent = _prim.GetParent();
      pxr::UsdModelAPI(parent).GetKind(&kindOfSchema);
    }

    if ((_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()
      || pxr::KindRegistry::IsA(kindOfSchema, pxr::KindTokens->model))
      && !collisionEnabled)
    {
      double metersPerUnit = data.second->MetersPerUnit();

      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        ParseSphere(_prim, geom, _scale, metersPerUnit);
        vis.SetName("visual_sphere");
        vis.SetGeom(geom);
        _link->AddVisual(vis);
      }
      else if (_prim.IsA<pxr::UsdGeomCylinder>())
      {
        ParseCylinder(_prim, geom, _scale, metersPerUnit);
        vis.SetName("visual_cylinder");
        vis.SetGeom(geom);
        _link->AddVisual(vis);
      }
      else if (_prim.IsA<pxr::UsdGeomCube>())
      {
        ParseCube(_prim, geom, _scale, metersPerUnit);
        vis.SetName("visual_box");
        vis.SetGeom(geom);
        _link->AddVisual(vis);
      }
      else if (_prim.IsA<pxr::UsdGeomMesh>())
      {
        ignition::math::Pose3d poseTmp;
        errors = ParseMesh(
          _prim, &_link.value(), vis, geom, _scale, _usdData, poseTmp);
        if (!errors.empty())
        {
          errors.emplace_back(UsdError(
          sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "Error parsing mesh"));
          return errors;
        }
      }
    }

    pxr::TfTokenVector schemasCollision = _prim.GetAppliedSchemas();
    bool physxCollisionAPIenable = false;
    for (const auto & token : schemasCollision)
    {
      if (std::string(token.GetText()) == "PhysxCollisionAPI")
      {
        physxCollisionAPIenable = true;
        break;
      }
    }

    if (collisionEnabled || physxCollisionAPIenable)
    {
      sdf::Collision col;

      // add _collision extension
      std::string collisionName = _prim.GetPath().GetName() + "_collision";
      col.SetName(collisionName);
      sdf::Geometry colGeom;

      ignition::math::Pose3d poseCol;
      ignition::math::Vector3d scaleCol(1, 1, 1);
      std::string linkName = pxr::TfStringify(_prim.GetPath());
      auto found = linkName.find(_link->Name());
      if (found != std::string::npos)
      {
        linkName = linkName.substr(0, found + _link->Name().size());
      }
      GetTransform(_prim, _usdData, poseCol, scaleCol, linkName);

      scaleCol *= _scale;

      double metersPerUnit = data.second->MetersPerUnit();

      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        ParseSphere(_prim, colGeom, scaleCol, metersPerUnit);
        col.SetGeom(colGeom);
        col.SetRawPose(poseCol);
      }
      else if (_prim.IsA<pxr::UsdGeomCylinder>())
      {
        ParseCylinder(_prim, colGeom, scaleCol, metersPerUnit);
        col.SetGeom(colGeom);
        col.SetRawPose(poseCol);
      }
      else if (_prim.IsA<pxr::UsdGeomCube>())
      {
        ParseCube(_prim, colGeom, scaleCol, metersPerUnit);
        col.SetGeom(colGeom);
        col.SetRawPose(poseCol);
      }
      else if (_prim.IsA<pxr::UsdGeomMesh>())
      {
        sdf::Visual visTmp;
        ignition::math::Pose3d poseTmp;
        errors = ParseMesh(
          _prim, &_link.value(), visTmp, colGeom, scaleCol, _usdData, poseTmp);
        if (!errors.empty())
        {
          errors.emplace_back(UsdError(
          sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "Error parsing mesh"));
          return errors;
        }
        col.SetRawPose(poseTmp);
        col.SetGeom(colGeom);
      }
      else if (primType == "Plane")
      {
        sdf::Plane plane;
        colGeom.SetType(sdf::GeometryType::PLANE);
        plane.SetSize(ignition::math::Vector2d(100, 100));
        colGeom.SetPlaneShape(plane);

        ignition::math::Pose3d pose;
        ignition::math::Vector3d scale(1, 1, 1);
        GetTransform(
          _prim, _usdData, pose, scale, pxr::TfStringify(_prim.GetPath()));
        col.SetRawPose(pose);
        col.SetGeom(colGeom);
      }

      _link->AddCollision(col);
    }

  }
  return errors;
}
}
}
}
