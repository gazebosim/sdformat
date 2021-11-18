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

#include "pxr/usd/usdPhysics/massAPI.h"
#include "pxr/usd/usdPhysics/collisionAPI.h"
#include "pxr/usd/usdPhysics/rigidBodyAPI.h"

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Mesh.hh"
#include "sdf/Sphere.hh"

#include "utils.hh"

#include "pxr/usd/usdGeom/metrics.h"

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
    std::map<std::string, std::shared_ptr<ignition::common::Material>> &_materials,
    const ignition::math::Vector3d &_scale,
    const double &_metersPerUnit)
  {
    int numSubMeshes = 0;

    for (const auto & child : _prim.GetChildren())
    {
      if (child.IsA<pxr::UsdGeomSubset>())
      {
        std::string nameMaterial = ParseMaterialName(child);
        auto it = _materials.find(nameMaterial);
        if (it != _materials.end())
        {
          link->visual_array_material.push_back(it->second);
        }
        link->visual_array_material_name.push_back(nameMaterial);

        ignition::common::Mesh meshSubset;

        numSubMeshes++;

        ignition::common::SubMesh subMeshSubset;
        subMeshSubset.SetPrimitiveType(ignition::common::SubMesh::TRISTRIPS);
        subMeshSubset.SetName("subgeommesh");

        meshSubset.AddMaterial(it->second);
        subMeshSubset.SetMaterialIndex(meshSubset.MaterialCount() - 1);

        pxr::VtIntArray faceVertexIndices;
        child.GetAttribute(pxr::TfToken("indices")).Get(&faceVertexIndices);
        std::cerr << "faceVertexIndices " << faceVertexIndices.size() << '\n';
        for (unsigned int i = 0; i < faceVertexIndices.size(); ++i)
        {
          subMeshSubset.AddIndex(_subMesh.Index(faceVertexIndices[i] * 3));
          subMeshSubset.AddIndex(_subMesh.Index(faceVertexIndices[i] * 3 + 1));
          subMeshSubset.AddIndex(_subMesh.Index(faceVertexIndices[i] * 3 + 2));
        }

        for (int i = 0; i < _subMesh.VertexCount(); ++i)
        {
          subMeshSubset.AddVertex(_subMesh.Vertex(i));
        }
        for (int i = 0; i < _subMesh.NormalCount(); ++i)
        {
          subMeshSubset.AddNormal(_subMesh.Normal(i));
        }
        for (int i = 0; i < _subMesh.TexCoordCount(); ++i)
        {
          subMeshSubset.AddTexCoord(_subMesh.TexCoord(i));
        }
        meshSubset.AddSubMesh(subMeshSubset);
        std::shared_ptr<sdf::Visual> visSubset;
        visSubset = std::make_shared<sdf::Visual>();
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
        std::cerr << "link->name " << link->name << '\n';
        GetTransform(child, _metersPerUnit, pose, scale, link->name);
        // link->pose = pose;
        // vis->SetRawPose(pose);
        link->visual_array.push_back(visSubset);
      }
    }
    return numSubMeshes;
  }

  void ParseMesh(
    const pxr::UsdPrim &_prim,
    LinkSharedPtr &link,
    std::shared_ptr<sdf::Visual> &_vis,
    const sdf::Material &_material,
    std::map<std::string, sdf::Material> &_materialsSDF,
    sdf::Geometry &_geom,
    std::map<std::string, std::shared_ptr<ignition::common::Material>> &_materials,
    const ignition::math::Vector3d &_scale,
    const double &_metersPerUnit)
  {
    if (pxr::UsdGeomGetStageUpAxis(_prim.GetStage()) == pxr::UsdGeomTokens->z) {
      std::cerr << "TEST Z" << '\n';
    }
    if (pxr::UsdGeomGetStageUpAxis(_prim.GetStage()) == pxr::UsdGeomTokens->y) {
      std::cerr << "TEST Y" << '\n';
    }


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

    unsigned int indexVertex = 0;
    for (unsigned int i = 0; i < faceVertexCounts.size(); ++i)
    {
      if (faceVertexCounts[i] == 3)
      {
        unsigned int j = indexVertex;
        unsigned int indexVertexStop = indexVertex + 3;
        for (; j < indexVertexStop; ++j)
        {
          subMesh.AddIndex(faceVertexIndices[j]);
          ++indexVertex;
        }
      }
      else if (faceVertexCounts[i] == 4)
      {
        subMesh.AddIndex(faceVertexIndices[indexVertex]);
        subMesh.AddIndex(faceVertexIndices[indexVertex + 1]);
        subMesh.AddIndex(faceVertexIndices[indexVertex + 2]);

        subMesh.AddIndex(faceVertexIndices[indexVertex]);
        subMesh.AddIndex(faceVertexIndices[indexVertex + 2]);
        subMesh.AddIndex(faceVertexIndices[indexVertex + 3]);

        indexVertex+=4;
      }
      // TODO(ahcorde): 5? 6?, 7?...
    }

    std::cerr << "textCoords " << textCoords.size() << '\n';
    for (auto & textCoord: textCoords)
    {
      subMesh.AddTexCoord(textCoord[0], textCoord[1]);
    }

    for (auto & point: points)
    {
      subMesh.AddVertex(
        point[0] * _metersPerUnit,
        point[1] * _metersPerUnit,
        point[2] * _metersPerUnit);
    }

    for (auto & normal: normals)
    {
      subMesh.AddNormal(normal[0], normal[1], normal[2]);
    }

    sdf::Mesh meshGeom;
    _geom.SetType(sdf::GeometryType::MESH);

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale(1, 1, 1);
    GetTransform(_prim, _metersPerUnit, pose, scale, link->name);
    meshGeom.SetScale(scale);
    _vis->SetRawPose(pose);
    //
    // auto parent = _prim.GetParent();
    // if (parent.IsA<pxr::UsdGeomMesh>())
    // {
    //   _vis->SetRawPose(pose);
    // }
    // else
    // {
    //   pxr::UsdPrimCompositionQuery query =
    //     pxr::UsdPrimCompositionQuery::GetDirectReferences(parent);
    //
    //   std::vector<pxr::UsdPrimCompositionQueryArc> arcs =
    //     query.GetCompositionArcs();
    //   bool isAPropReference = false;
    //   for (auto & a : arcs )
    //   {
    //     pxr::SdfLayerHandle handler = a.GetIntroducingLayer();
    //     std::set<std::string> ss = handler->GetExternalReferences();
    //     for (auto & s : ss)
    //     {
    //       if (s.find("Prop") != std::string::npos &&
    //           parent.GetPath().GetName() == "geometry")
    //       {
    //         isAPropReference = true;
    //         break;
    //       }
    //     }
    //   }
    //   if (isAPropReference)
    //   {
    //     _vis->SetRawPose(pose);
    //   }
    // }

    std::string primName = pxr::TfStringify(_prim.GetPath());

    _vis->SetName("_" + ignition::common::basename(primName) + "_");
    _vis->SetMaterial(_material);

    std::string directoryMesh = directoryFromUSDPath(primName) + primName;

    std::cerr << "directoryMesh " << directoryMesh + ".dae" << '\n';
    meshGeom.SetFilePath(
      ignition::common::joinPaths(
        directoryMesh, ignition::common::basename(directoryMesh)) + ".dae");

    int numSubMeshes = ParseMeshSubGeom(
      _prim, link, subMesh, meshGeom, _materials, _scale, _metersPerUnit);

    _geom.SetMeshShape(meshGeom);
    _vis->SetGeom(_geom);

    if (numSubMeshes == 0)
    {
      std::string nameMaterial = ParseMaterialName(_prim);
      auto it = _materials.find(nameMaterial);
      auto itSDF = _materialsSDF.find(nameMaterial);
      if (it != _materials.end() && itSDF != _materialsSDF.end())
      {
        _vis->SetMaterial(itSDF->second);
        link->visual_array_material.push_back(it->second);
        mesh.AddMaterial(it->second);
        subMesh.SetMaterialIndex(mesh.MaterialCount() - 1);
        std::cerr << " \t Setted material " << nameMaterial << " to " << primName << '\n';
      }
      link->visual_array.push_back(_vis);

      link->visual_array_material_name.push_back(nameMaterial);
      mesh.AddSubMesh(subMesh);

      if (ignition::common::createDirectories(directoryMesh))
      {
        directoryMesh = ignition::common::joinPaths(directoryMesh, ignition::common::basename(directoryMesh));
        // Export with extension
        ignition::common::ColladaExporter exporter;
        exporter.Export(&mesh, directoryMesh, false);
      }
    }
  }

  void ParseCube(const pxr::UsdPrim &_prim, std::shared_ptr<sdf::Visual> &_vis,
    const sdf::Material &_material,
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
    _vis->SetName("visual_box");
    _vis->SetGeom(_geom);
    _vis->SetMaterial(_material);
  }

  void ParseSphere(const pxr::UsdPrim &_prim, std::shared_ptr<sdf::Visual> &_vis,
    const sdf::Material &_material,
    sdf::Geometry &_geom, ignition::math::Vector3d &_scale,
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
    _vis->SetName("visual_sphere");
    _vis->SetGeom(_geom);
    _vis->SetMaterial(_material);
  }

  void ParseCylinder(
    const pxr::UsdPrim &_prim,
    std::shared_ptr<sdf::Visual> &_vis,
    const sdf::Material &_material,
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
    c.SetLength(height * _metersPerUnit * _scale.X());

    sdferr << "this is a cylinder r: "
      << radius * _metersPerUnit * _scale.X()
      << " height :" << height * _metersPerUnit * _scale.X() << "\n";

    _geom.SetCylinderShape(c);
    _vis->SetName("visual_cylinder");
    _vis->SetGeom(_geom);
    _vis->SetMaterial(_material);
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

  LinkSharedPtr ParseLinks(
    const pxr::UsdPrim &_prim,
    LinkSharedPtr &link,
    std::map<std::string, std::shared_ptr<ignition::common::Material>> &_materials,
    std::map<std::string, sdf::Material> &_materialsSDF,
    const double _metersPerUnit,
    int &_skip)
  {
    std::cerr << "************ ADDED LINK ************" << '\n';

    pxr::TfToken axis = pxr::UsdGeomGetStageUpAxis(_prim.GetStage());
    // _prim.GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
    std::string upAxis = axis.GetText();
    std::cerr << " ->>>>>>>> upAxis " << upAxis << '\n';

    pxr::UsdPrimCompositionQuery query =
      pxr::UsdPrimCompositionQuery::GetDirectReferences(_prim);

    std::vector<pxr::UsdPrimCompositionQueryArc> arcs =
      query.GetCompositionArcs();
    bool isAPropReference = false;
    for (auto & a : arcs )
    {
      pxr::SdfLayerHandle handler = a.GetIntroducingLayer();
      auto stage = pxr::UsdStage::Open(handler);
      if (stage)
      {
        stage->GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
        upAxis = axis.GetText();
        std::cerr << " ->>>>>>>> upAxis " << upAxis << '\n';
      }
    }

    bool newlink = false;

    if (link == nullptr)
    {
      link.reset(new Link);
      link->clear();
      newlink = true;
    }

    std::string primName = pxr::TfStringify(_prim.GetPath());
    removeSubStr(primName, "/World");

    std::vector<std::string> tokens = ignition::common::split(primName, "/");
    if (tokens.size() >= 3)
    {
      link->name = pxr::TfStringify("/" + tokens[0] + "/" + tokens[1]);
    }
    else
    {
      link->name = pxr::TfStringify(_prim.GetPath());
    }

    if (!link->inertial)
    {
      link->inertial = std::make_shared<ignition::math::Inertiald>();
      getInertial(_prim, link);
    }

    if (newlink)
    {
      std::string originalPrimName = pxr::TfStringify(_prim.GetPath());
      size_t pos = std::string::npos;
      std::string name;
      if ((pos = originalPrimName.find(link->name) )!= std::string::npos)
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
      std::cerr << "PaserLinks" << '\n';
          ignition::math::Pose3d pose;
          ignition::math::Vector3d scale(1, 1, 1);
          GetTransform(tmpPrim, _metersPerUnit, pose, scale, "");
          link->pose = pose;
          link->scale = scale;
      std::cerr << "PaserLinks" << '\n';
    }

    sdf::Geometry geom;
    if (_prim.IsA<pxr::UsdGeomSphere>() ||
        _prim.IsA<pxr::UsdGeomCylinder>() ||
        _prim.IsA<pxr::UsdGeomCube>() ||
        _prim.IsA<pxr::UsdGeomMesh>())
    {
      std::shared_ptr<sdf::Visual> vis;
      vis = std::make_shared<sdf::Visual>();

      auto variant_geom = pxr::UsdGeomGprim(_prim);

      auto transforms = variant_geom.GetXformOpOrderAttr();

      if (!transforms)
      {
        sdferr << "No transforms available!\n";
      }
      //
      // std::tuple<pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf, bool, bool, bool, bool> transformsTuple =
      //   ParseTransform(_prim);
      // pxr::GfVec3f scale = std::get<0>(transformsTuple);
      // pxr::GfVec3f translate = std::get<1>(transformsTuple);
      // pxr::GfQuatf rotationQuad = std::get<2>(transformsTuple);
      //
      // bool isScale = std::get<3>(transformsTuple);
      // bool isTranslate = std::get<4>(transformsTuple);
      // bool isRotation = std::get<5>(transformsTuple);
      //
      //
      // if (isScale)
      //   link->scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
      // if (isTranslate && isRotation)
      // {
      //   vis->SetRawPose(
      //     ignition::math::Pose3d(
      //       ignition::math::Vector3d(
      //         translate[0] * _metersPerUnit,
      //         translate[1] * _metersPerUnit,
      //         translate[2] * _metersPerUnit),
      //       ignition::math::Quaterniond(
      //         rotationQuad.GetReal(),
      //         rotationQuad.GetImaginary()[0],
      //         rotationQuad.GetImaginary()[1],
      //         rotationQuad.GetImaginary()[2])));
      // }
      // else
      // {
      //   vis->SetRawPose(link->pose);
      //   link->pose = ignition::math::Pose3d();
      // }

      sdf::Material material = ParseMaterial(_prim);

      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        ParseSphere(_prim, vis, material, geom, link->scale, _metersPerUnit);
      }
      else if (_prim.IsA<pxr::UsdGeomCylinder>())
      {
        ParseCylinder(_prim, vis, material, geom, link->scale, _metersPerUnit);
      }
      else if (_prim.IsA<pxr::UsdGeomCube>())
      {
        ParseCube(_prim, vis, material, geom, link->scale, _metersPerUnit);
      }
      else if (_prim.IsA<pxr::UsdGeomMesh>())
      {
        ParseMesh(_prim, link, vis, material, _materialsSDF, geom, _materials, link->scale, _metersPerUnit);
      }
    //
    //   pxr::TfTokenVector schemas = _prim.GetAppliedSchemas();
    //   int isOnlyCollision = 0;
    //   for (auto & token : schemas)
    //   {
    //     std::cerr << "GetText " << token.GetText() << '\n';
    //     if (std::string(token.GetText()) == "PhysicsCollisionAPI" ||
    //         std::string(token.GetText()) == "PhysxCollisionAPI" )
    //     {
    //       isOnlyCollision = 2;
    //     }
    //   }
    //
    //   std::cerr << "isOnlyCollision " << isOnlyCollision << '\n';
    //   if (isOnlyCollision != 2 && !_prim.IsA<pxr::UsdGeomMesh>())
    //   {
    //     link->visual_array.push_back(vis);
    //     link->visual_array_material_name.push_back("");
    //   }
    // }
    //
    // bool collisionEnabled = false;
    // if (_prim.HasAPI<pxr::UsdPhysicsCollisionAPI>())
    // {
    //   std::cerr << "UsdPhysicsCollisionAPI" << '\n';
    //
    //   _prim.GetAttribute(pxr::TfToken("physics:collisionEnabled")).Get(&collisionEnabled);
    // }
    //
    // if (collisionEnabled)
    // {
    //   sdf::Collision col;
    //
    //   // add _collision extension
    //   std::string collisionName = link->name + kCollisionExt;
    //   col.SetName(collisionName);
    //   col.SetGeom(geom);
    //
    //   std::tuple<pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf, bool, bool, bool, bool> transformsTuple =
    //     ParseTransform(_prim);
    //
    //   pxr::GfVec3f scale = std::get<0>(transformsTuple);
    //   pxr::GfVec3d translate = std::get<1>(transformsTuple);
    //   pxr::GfQuatd rotationQuad = std::get<2>(transformsTuple);
    //
    //   bool isScale = std::get<3>(transformsTuple);;
    //   bool isTranslate = std::get<4>(transformsTuple);;
    //   bool isRotation = std::get<5>(transformsTuple);;
    //
    //   if (isTranslate)
    //   {
    //     col.SetRawPose(
    //       ignition::math::Pose3d(
    //         ignition::math::Vector3d(
    //           translate[0] * _metersPerUnit,
    //           translate[1] * _metersPerUnit,
    //           translate[2] * _metersPerUnit),
    //         col.RawPose().Rot()));
    //   }
    //   if (isRotation)
    //   {
    //     col.SetRawPose(
    //       ignition::math::Pose3d(
    //         col.RawPose().Pos(),
    //         ignition::math::Quaterniond(
    //           rotationQuad.GetReal(),
    //           rotationQuad.GetImaginary()[0],
    //           rotationQuad.GetImaginary()[1],
    //           rotationQuad.GetImaginary()[2])));
    //   }
    //
    //   std::shared_ptr<sdf::Collision> colPtr = std::make_shared<sdf::Collision>();
    //   colPtr->SetRawPose(col.RawPose());
    //   colPtr->SetGeom(geom);
    //   colPtr->SetName(col.Name());
    //
    //   if (_prim.IsA<pxr::UsdGeomCube>())
    //   {
    //     if (isScale)
    //     {
    //       const sdf::Box * box = colPtr->Geom()->BoxShape();
    //       sdf::Box * boxEditable = const_cast<sdf::Box*>(box);
    //       ignition::math::Vector3d size = box->Size();
    //       boxEditable->SetSize(ignition::math::Vector3d(
    //         size.X() * scale[0],
    //         size.Y() * scale[1],
    //         size.Z() * scale[2]));
    //     }
    //   }
    //
    //   link->collision_array.push_back(colPtr);
    //   // Collision (optional)
    //   // Assign the first collision to the .collision ptr, if it exists
    //   if (!link->collision_array.empty())
    //     link->collision = link->collision_array[0];
    }

    // Visual (optional)
    // Assign the first visual to the .visual ptr, if it exists
    if (link != nullptr) {
      if (!link->visual_array.empty())
        link->visual = link->visual_array[0];
    }
    else
    {
      std::cerr << "Link nullptr" << '\n';
    }

    return link;
  }
}
