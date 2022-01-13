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

#include "links.hh"

#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include "pxr/usd/usd/primCompositionQuery.h"
#include <pxr/usd/usd/stage.h>

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"

#include "sdf/Console.hh"

#include "ignition/common/ColladaExporter.hh"
#include "ignition/common/Filesystem.hh"
#include "ignition/common/Mesh.hh"
#include "ignition/common/SubMesh.hh"
#include "ignition/common/Util.hh"

#include "ignition/math/Matrix4.hh"

#include "ignition/common/GTSMeshUtils.hh"

#include "pxr/usd/usdPhysics/massAPI.h"
#include "pxr/usd/usdPhysics/collisionAPI.h"
#include "pxr/usd/usdPhysics/rigidBodyAPI.h"

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Mesh.hh"
#include "sdf/Sphere.hh"
#include "sdf/Conversions.hh"

#include "utils.hh"
#include "polygon_helper.hh"

#include "pxr/usd/usdGeom/metrics.h"

#include "usd/USDStage.hh"

namespace usd
{
  const char kCollisionExt[] = "_collision";

  std::string ParseMaterialName(const pxr::UsdPrim &_prim)
  {
    // Materials
    std::string nameMaterial;

    auto bindMaterial = pxr::UsdShadeMaterialBindingAPI(_prim);
    pxr::UsdRelationship directBindingRel =
      bindMaterial.GetDirectBindingRel();

    pxr::SdfPathVector paths;
    directBindingRel.GetTargets(&paths);
    for (auto p: paths)
    {
      std::vector<std::string> tokensMaterial =
        ignition::common::split(pxr::TfStringify(p), "/");

      if(tokensMaterial.size() > 0)
      {
        nameMaterial = tokensMaterial[tokensMaterial.size() - 1];
      }
    }
    std::cerr << "nameMaterial " << nameMaterial << '\n';
    return nameMaterial;
  }

  int ParseMeshSubGeom(const pxr::UsdPrim &_prim,
    LinkSharedPtr &link,
    ignition::common::SubMesh &_subMesh,
    sdf::Mesh &_meshGeom,
    const ignition::math::Vector3d &_scale,
    USDData &_usdData)
  {
    std::pair<std::string, std::shared_ptr<USDStage>> data =
      _usdData.findStage(_prim.GetPath().GetName());

    int numSubMeshes = 0;

    for (const auto & child : _prim.GetChildren())
    {
      if (child.IsA<pxr::UsdGeomSubset>())
      {
        std::shared_ptr<sdf::Visual> visSubset;
        visSubset = std::make_shared<sdf::Visual>();

        std::string nameMaterial = ParseMaterialName(child);
        auto it = _usdData.materials.find(nameMaterial);
        if (it != _usdData.materials.end())
        {
          std::cerr << "material found" << '\n';
          visSubset->SetMaterial(it->second);
        }
        else
        {
          std::cerr << "material NOT found" << '\n';
        }

        ignition::common::Mesh meshSubset;

        numSubMeshes++;

        ignition::common::SubMesh subMeshSubset;
        subMeshSubset.SetPrimitiveType(ignition::common::SubMesh::TRISTRIPS);
        subMeshSubset.SetName("subgeommesh");

        if (it != _usdData.materials.end())
        {
          std::shared_ptr<ignition::common::Material> matCommon = convertMaterial(it->second);
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
        std::string directoryMesh = directoryFromUSDPath(childPathName) + childPathName;

        if (ignition::common::createDirectories(directoryMesh))
        {
          directoryMesh = ignition::common::joinPaths(directoryMesh, ignition::common::basename(directoryMesh));

          // Export with extension
          ignition::common::ColladaExporter exporter;
          exporter.Export(&meshSubset, directoryMesh, false);
        }
        std::cerr << "directoryMesh " << directoryMesh + ".dae" << '\n';
        _meshGeom.SetFilePath(directoryMesh + ".dae");

        geomSubset.SetMeshShape(_meshGeom);
        visSubset->SetName("mesh_subset");
        visSubset->SetGeom(geomSubset);

        ignition::math::Pose3d pose;
        ignition::math::Vector3d scale(1, 1, 1);
        GetTransform(child, _usdData, pose, scale, link->name);
        // link->pose = pose;
        visSubset->SetRawPose(pose);
        link->visual_array.push_back(visSubset);
      }
    }
    return numSubMeshes;
  }

  ignition::math::Pose3d ParseMesh(
    const pxr::UsdPrim &_prim,
    LinkSharedPtr &link,
    std::shared_ptr<sdf::Visual> &_vis,
    sdf::Geometry &_geom,
    const ignition::math::Vector3d &_scale,
    USDData &_usdData)
  {
    // if (pxr::UsdGeomGetStageUpAxis(_prim.GetStage()) == pxr::UsdGeomTokens->z) {
    //   std::cerr << "TEST Z" << '\n';
    // }
    // if (pxr::UsdGeomGetStageUpAxis(_prim.GetStage()) == pxr::UsdGeomTokens->y) {
    //   std::cerr << "TEST Y" << '\n';
    // }

    // auto variantMesh = pxr::UsdGeomMesh(_prim);
    // pxr::TfToken axis;
    // auto orientation = variantMesh.GetOrientationAttr();
    // orientation.Get(&axis);
    // std::cerr << "axis : " << axis.GetText() << '\n';
    // std::cerr << "axis : " << pxr::UsdGeomGetFallbackUpAxis().GetText() << '\n';
    // std::cerr << "axis : " << pxr::UsdGeomGetFallbackUpAxis().GetText() << '\n';

    // axis = pxr::UsdGeomGetStageUpAxis(_prim.GetStage());
    // // _prim.GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
    // std::string upAxis = axis.GetText();
    // std::cerr << "upAxis " << upAxis << '\n';

    std::pair<std::string, std::shared_ptr<USDStage>> data =
      _usdData.findStage(_prim.GetPath().GetName());

    double metersPerUnit = data.second->_metersPerUnit;

    std::cerr << "/* UsdGeomMesh */" << '\n';
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

    std::vector<unsigned int> indexes = PolygonToTriangles(
      faceVertexIndices, faceVertexCounts, points);
    for (unsigned int i = 0; i < indexes.size(); ++i)
    {
      subMesh.AddIndex(indexes[i]);
    }

    for (auto & textCoord: textCoords)
    {
      subMesh.AddTexCoord(textCoord[0], (1 - textCoord[1]));
    }

    bool upAxisZ = (data.second->_upAxis == "Z");
    ignition::math::Matrix4d m(
      1, 0, 0, 0,
      0, 0, -1, 0,
      0, 1, 0, 0,
      0, 0, 0, 1);
    for (auto & point: points)
    {
      ignition::math::Vector3d v =
        ignition::math::Vector3d(point[0], point[1], point[2]) * metersPerUnit;
      if(upAxisZ)
      {
        subMesh.AddVertex(v);
      }
      else
      {
        // std::cerr << "v " << v << '\n';
        // v = m * v;
        // std::cerr << "v zup" << v << '\n';
        subMesh.AddVertex(v);
      }
    }

    for (auto & normal: normals)
    {
      subMesh.AddNormal(normal[0], normal[1], normal[2]);
    }

    // std::cerr << "\t\t _prim " << pxr::TfStringify(_prim.GetPath()) << '\n';
    // std::cerr << "\t\t _prim " << _prim.GetPath().GetName() << '\n';
    // std::cerr << "\t\t normals " << normals.size() << '\n';
    // std::cerr << "\t\t points " << points.size() << '\n';
    // std::cerr << "\t\t textCoords " << textCoords.size() << '\n';
    // std::cerr << "\t\t faceVertexIndices " << faceVertexIndices.size() << '\n';
    // std::cerr << "\t\t faceVertexCounts " << faceVertexCounts.size() << '\n';

    sdf::Mesh meshGeom;
    _geom.SetType(sdf::GeometryType::MESH);

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale(1, 1, 1);
    size_t nSlash = std::count(link->name.begin(), link->name.end(), '/');
    if (nSlash == 1)
    {
      GetTransform(_prim, _usdData, pose, scale, "/");
    }
    else
    {
      GetTransform(_prim, _usdData, pose, scale, link->name);
    }
    meshGeom.SetScale(scale * link->scale);

    std::string primName = pxr::TfStringify(_prim.GetPath());

    std::string directoryMesh = directoryFromUSDPath(primName) + primName;

    std::cerr << "directoryMesh " << directoryMesh + ".dae" << '\n';
    meshGeom.SetFilePath(
      ignition::common::joinPaths(
        directoryMesh, ignition::common::basename(directoryMesh)) + ".dae");

    int numSubMeshes = ParseMeshSubGeom(
      _prim, link, subMesh, meshGeom, _scale, _usdData);

    _geom.SetMeshShape(meshGeom);

    if (numSubMeshes == 0)
    {
      if (_vis != nullptr)
      {
        std::string nameMaterial = ParseMaterialName(_prim);
        auto it = _usdData.materials.find(nameMaterial);
        if (it != _usdData.materials.end())
        {
          _vis->SetMaterial(it->second);
          // link->visual_array_material.push_back(it->second);
          std::shared_ptr<ignition::common::Material> matCommon = convertMaterial(it->second);
          mesh.AddMaterial(matCommon);
          subMesh.SetMaterialIndex(mesh.MaterialCount() - 1);
          std::cerr << " \t Setted material " << nameMaterial << " to " << primName << '\n';
        }
        link->visual_array.push_back(_vis);
      }

      // link->visual_array_material_name.push_back(nameMaterial);
      mesh.AddSubMesh(subMesh);

      if (ignition::common::createDirectories(directoryMesh))
      {
        directoryMesh = ignition::common::joinPaths(directoryMesh, ignition::common::basename(directoryMesh));
        // Export with extension
        ignition::common::ColladaExporter exporter;
        exporter.Export(&mesh, directoryMesh, false);
      }
    }
    return pose;
  }

  void ParseCube(const pxr::UsdPrim &_prim,
    sdf::Geometry &_geom,
    ignition::math::Vector3d &_scale,
    const double _metersPerUnit)
  {
    double size;
    auto variant_cylinder = pxr::UsdGeomCube(_prim);
    variant_cylinder.GetSizeAttr().Get(&size);

    size = size * _metersPerUnit;

    sdf::Box box;
    _geom.SetType(sdf::GeometryType::BOX);
    box.SetSize(ignition::math::Vector3d(
      size * _scale.X(),
      size * _scale.Y(),
      size * _scale.Z()));

    sdferr << "this is a cube" << box.Size() << "\n";

    _geom.SetBoxShape(box);
  }

  void ParseSphere(const pxr::UsdPrim &_prim,
    sdf::Geometry &_geom,
    ignition::math::Vector3d &_scale,
    const double _metersPerUnit)
  {
    double radius;
    auto variant_sphere = pxr::UsdGeomSphere(_prim);
    variant_sphere.GetRadiusAttr().Get(&radius);
    sdferr << "This is a sphere r: " << radius * _metersPerUnit * _scale.X() << "\n";

    sdf::Sphere s;
    _geom.SetType(sdf::GeometryType::SPHERE);
    s.SetRadius(radius * _metersPerUnit * _scale.X());
    _geom.SetSphereShape(s);
  }

  void ParseCylinder(
    const pxr::UsdPrim &_prim,
    sdf::Geometry &_geom,
    const ignition::math::Vector3d &_scale,
    const double _metersPerUnit)
  {
    auto variant_cylinder = pxr::UsdGeomCylinder(_prim);
    double radius;
    double height;
    variant_cylinder.GetRadiusAttr().Get(&radius);
    variant_cylinder.GetHeightAttr().Get(&height);

    sdf::Cylinder c;
    _geom.SetType(sdf::GeometryType::CYLINDER);

    c.SetRadius(radius * _metersPerUnit * _scale.X());
    c.SetLength(height * _metersPerUnit * _scale.Y());

    sdferr << "this is a cylinder r: "
      << radius * _metersPerUnit * _scale.X()
      << " height :" << height * _metersPerUnit * _scale.X() << "\n";

    _geom.SetCylinderShape(c);
  }

  void getInertial(const pxr::UsdPrim &_prim, LinkSharedPtr &link)
  {
    float mass;
    pxr::GfVec3f centerOfMass;
    pxr::GfVec3f diagonalInertia;

    ignition::math::MassMatrix3d massMatrix;

    if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
    {
      std::cerr << "UsdPhysicsMassAPI " <<  pxr::TfStringify(_prim.GetPath()) << '\n';
      if (pxr::UsdAttribute massAttribute =
        _prim.GetAttribute(pxr::TfToken("physics:mass")))
      {
        massAttribute.Get(&mass);
        std::cerr << "MASS " << mass << '\n';

        if (pxr::UsdAttribute centerOfMassAttribute =
          _prim.GetAttribute(pxr::TfToken("physics:centerOfMass"))) {
          centerOfMassAttribute.Get(&centerOfMass);

        }
        if (pxr::UsdAttribute diagonalInertiaAttribute =
          _prim.GetAttribute(pxr::TfToken("physics:diagonalInertia"))) {
          diagonalInertiaAttribute.Get(&diagonalInertia);
        }

        if (mass < 0.0001)
        {
          mass = 0.1;
        }

        std::cerr << "centerOfMass " << centerOfMass << '\n';
        std::cerr << "diagonalInertia " << diagonalInertia << '\n';

        massMatrix.SetMass(mass);
        massMatrix.SetDiagonalMoments(
          ignition::math::Vector3d(
            diagonalInertia[0],
            diagonalInertia[1],
            diagonalInertia[2]));

        link->inertial->SetPose(ignition::math::Pose3d(
            ignition::math::Vector3d(
              centerOfMass[0], centerOfMass[1], centerOfMass[2]),
            ignition::math::Quaterniond(1.0, 0, 0, 0)));

        link->inertial->SetMassMatrix(massMatrix);
      }
    }
    else
    {
      auto parent = _prim.GetParent();
      if (parent)
      {
        getInertial(parent, link);
      }
    }
  }

  std::string ParseLinks(
    const pxr::UsdPrim &_prim,
    const std::string &_nameLink,
    LinkSharedPtr &link,
    USDData &_usdData,
    int &_skip)
  {
    std::cerr << "************ ADDED LINK ************" << '\n';

    // pxr::TfToken axis = pxr::UsdGeomGetStageUpAxis(_prim.GetStage());
    // // _prim.GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
    // std::string upAxis = axis.GetText();
    // std::cerr << " ->>>>>>>> upAxis " << upAxis << '\n';
    //
    // pxr::UsdPrimCompositionQuery query =
    //   pxr::UsdPrimCompositionQuery::GetDirectReferences(_prim);
    //
    // std::vector<pxr::UsdPrimCompositionQueryArc> arcs =
    //   query.GetCompositionArcs();
    // bool isAPropReference = false;
    // for (auto & a : arcs )
    // {
    //   pxr::SdfLayerHandle handler = a.GetIntroducingLayer();
    //   auto stage = pxr::UsdStage::Open(handler);
    //   if (stage)
    //   {
    //     stage->GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
    //     upAxis = axis.GetText();
    //     std::cerr << " ->>>>>>>> upAxis " << upAxis << '\n';
    //   }
    // }

    std::pair<std::string, std::shared_ptr<USDStage>> data =
      _usdData.findStage(_prim.GetPath().GetName());

    double metersPerUnit = data.second->_metersPerUnit;

    bool newlink = false;

    if (link == nullptr)
    {
      link.reset(new Link);
      link->clear();
      newlink = true;
    }

    std::string primName = pxr::TfStringify(_prim.GetPath());
    removeSubStr(primName, "/World");

    if(link->name.empty())
    {
      std::vector<std::string> tokens = ignition::common::split(primName, "/");
      if (tokens.size() >= 3)
      {
        link->name = pxr::TfStringify("/" + tokens[0] + "/" + tokens[1]);
      }
      else
      {
        link->name = pxr::TfStringify(_prim.GetPath());
      }
    }

    std::string result = link->name;

    if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
    {
      result = primName;//pxr::TfStringify(_prim.GetPath());
      link->name = primName;
    }

    if (!link->inertial)
    {
      link->inertial = std::make_shared<ignition::math::Inertiald>();
      getInertial(_prim, link);
    }

    if (newlink)
    {
      link->name = _nameLink;

      std::cerr << "\t Creating a new Link! " << link->name << '\n';
      std::string originalPrimName = pxr::TfStringify(_prim.GetPath());
      size_t pos = std::string::npos;
      std::string name;
      if ((pos = originalPrimName.find(link->name))!= std::string::npos)
      {
        name = originalPrimName.erase(
          pos + link->name.length(), originalPrimName.length() - (pos + link->name.length()));
        std::cerr << "name " << name << '\n';
      }
      pxr::UsdPrim tmpPrim = _prim;
      if (!name.empty())
      {
        while(tmpPrim)
        {
          std::cerr << "pxr::TfStringify(tmpPrim.GetPath()) " << pxr::TfStringify(tmpPrim.GetPath()) << " == " << name << '\n';
          if (pxr::TfStringify(tmpPrim.GetPath()) == name)
          {
            break;
          }
          tmpPrim = tmpPrim.GetParent();
        }
      }
      ignition::math::Pose3d pose;
      ignition::math::Vector3d scale(1, 1, 1);
      GetTransform(tmpPrim, _usdData, pose, scale, "");
      size_t nSlash = std::count(link->name.begin(), link->name.end(), '/');
      if (nSlash > 1)
        link->pose = pose;
      link->scale = scale;
    }

    sdf::Geometry geom;
    if (_prim.IsA<pxr::UsdGeomSphere>() ||
        _prim.IsA<pxr::UsdGeomCylinder>() ||
        _prim.IsA<pxr::UsdGeomCube>() ||
        _prim.IsA<pxr::UsdGeomMesh>() ||
        std::string(_prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Plane")
    {
      std::shared_ptr<sdf::Visual> vis;
      vis = std::make_shared<sdf::Visual>();

      auto variant_geom = pxr::UsdGeomGprim(_prim);

      pxr::TfTokenVector schemas = _prim.GetAppliedSchemas();
      bool isPhysicsMeshCollisionAPI = false;
      // for (auto & token : schemas)
      // {
      //   std::cerr << "GetText " << token.GetText() << '\n';
      //   if (std::string(token.GetText()) == "PhysicsMeshCollisionAPI" )
      //   {
      //     isPhysicsMeshCollisionAPI = true;
      //     break;
      //   }
      // }

      bool collisionEnabled = false;
      if (_prim.HasAPI<pxr::UsdPhysicsCollisionAPI>())
      {
        std::cerr << "UsdPhysicsCollisionAPI" << '\n';

        _prim.GetAttribute(pxr::TfToken("physics:collisionEnabled")).Get(&collisionEnabled);
      }

      if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()
        || (!isPhysicsMeshCollisionAPI && !collisionEnabled))
      {
        sdf::Material material = ParseMaterial(_prim);

        if (_prim.IsA<pxr::UsdGeomSphere>())
        {
          ParseSphere(_prim, geom, link->scale, metersPerUnit);
          vis->SetName("visual_sphere");
          vis->SetGeom(geom);
          vis->SetMaterial(material);
        }
        else if (_prim.IsA<pxr::UsdGeomCylinder>())
        {
          ParseCylinder(_prim, geom, link->scale, metersPerUnit);
          vis->SetName("visual_cylinder");
          vis->SetGeom(geom);
          vis->SetMaterial(material);
        }
        else if (_prim.IsA<pxr::UsdGeomCube>())
        {
          ParseCube(_prim, geom, link->scale, metersPerUnit);
          vis->SetName("visual_box");
          vis->SetGeom(geom);
          vis->SetMaterial(material);
        }
        else if (_prim.IsA<pxr::UsdGeomMesh>())
        {
          vis->SetMaterial(material);
          ignition::math::Pose3d pose = ParseMesh(
            _prim, link, vis, geom, link->scale, _usdData);
          vis->SetName("_" + ignition::common::basename(primName) + "_");
          vis->SetRawPose(pose);
          vis->SetGeom(geom);
        }
      }

      pxr::TfTokenVector schemasCollision = _prim.GetAppliedSchemas();
      bool physxCollisionAPIenable = false;
      for (auto & token : schemasCollision)
      {
        std::cerr << "GetText " << token.GetText() << '\n';
        if (std::string(token.GetText()) == "PhysxCollisionAPI")
        {
          physxCollisionAPIenable = true;
        }
      }

      if (collisionEnabled || physxCollisionAPIenable)
      {
        sdf::Collision col;

        // add _collision extension
        std::string collisionName = _prim.GetPath().GetName() + kCollisionExt;
        col.SetName(collisionName);
        sdf::Geometry colGeom;

        ignition::math::Pose3d poseCol;
        ignition::math::Vector3d scaleCol(1, 1, 1);
        GetTransform(_prim, _usdData, poseCol, scaleCol, link->name);

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
          std::shared_ptr<sdf::Visual> visTmp = nullptr;
          ignition::math::Pose3d pose = ParseMesh(
            _prim, link, visTmp, colGeom, scaleCol, _usdData);
          col.SetRawPose(pose);
          col.SetGeom(colGeom);
        }
        else if (std::string(_prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Plane")
        {
          sdf::Plane plane;
          colGeom.SetType(sdf::GeometryType::PLANE);
          plane.SetSize(ignition::math::Vector2d(100, 100));
          colGeom.SetPlaneShape(plane);

          ignition::math::Pose3d pose;
          ignition::math::Vector3d scale(1, 1, 1);
          GetTransform(_prim, _usdData, pose, scale, pxr::TfStringify(_prim.GetPath()));
          col.SetRawPose(pose);
          std::cerr << "Add collision Plane" << '\n';
        }

        // ignition::math::Pose3d pose;
        // ignition::math::Vector3d scale(1, 1, 1);
        // GetTransform(_prim, _usdData, pose, scale, "");
        // col.SetRawPose(pose);

        std::shared_ptr<sdf::Collision> colPtr = std::make_shared<sdf::Collision>();
        colPtr->SetRawPose(col.RawPose());
        colPtr->SetGeom(colGeom);
        colPtr->SetName(col.Name());

        // if (_prim.IsA<pxr::UsdGeomCube>())
        // {
        //   const sdf::Box * box = colPtr->Geom()->BoxShape();
        //   sdf::Box * boxEditable = const_cast<sdf::Box*>(box);
        //   ignition::math::Vector3d size = box->Size();
        //   boxEditable->SetSize(ignition::math::Vector3d(
        //     size.X() * scale[0],
        //     size.Y() * scale[1],
        //     size.Z() * scale[2]));
        // }

        link->collision_array.push_back(colPtr);
        // Collision (optional)
        // Assign the first collision to the .collision ptr, if it exists
        if (!link->collision_array.empty())
          link->collision = link->collision_array[0];
      }

      // Visual (optional)
      // Assign the first visual to the .visual ptr, if it exists
      if (link != nullptr) {
        if (!link->visual_array.empty())
          link->visual = link->visual_array[0];
      }
    }
    else
    {
      std::cerr << "Link nullptr" << '\n';
    }

    return result;
  }
}
