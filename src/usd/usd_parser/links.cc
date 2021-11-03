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

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"

#include "sdf/Console.hh"

#include "ignition/common/ColladaExporter.hh"
#include "ignition/common/Filesystem.hh"
#include "ignition/common/Mesh.hh"
#include "ignition/common/SubMesh.hh"
#include "ignition/common/Util.hh"

#include "pxr/usd/usdPhysics/massAPI.h"
#include "pxr/usd/usdPhysics/collisionAPI.h"

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Mesh.hh"
#include "sdf/Sphere.hh"

#include "utils.hh"

namespace usd
{
  const char kCollisionExt[] = "_collision";

  LinkSharedPtr ParseLinks(const pxr::UsdPrim &_prim, LinkSharedPtr &link, const double _metersPerUnit)
  {
    std::cerr << "************ ADDED LINK ************" << '\n';

    if (link == nullptr)
    {
      link.reset(new Link);
      link->clear();
    }

    std::string primName = pxr::TfStringify(_prim.GetPath());
    size_t pos = std::string::npos;
    if ((pos  = primName.find("/World") )!= std::string::npos)
    {
      primName.erase(pos, std::string("/World").length());
    }

    std::vector<std::string> tokens = ignition::common::split(primName, "/");
    if (tokens.size() == 3)
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

      if (_prim.HasAPI<pxr::UsdPhysicsMassAPI>())
      {
        ignition::math::MassMatrix3d massMatrix;
        std::cerr << "UsdPhysicsMassAPI" << '\n';
        float mass;
        pxr::GfVec3f centerOfMass;
        pxr::GfVec3f diagonalInertia;
        _prim.GetAttribute(pxr::TfToken("physics:mass")).Get(&mass);
        _prim.GetAttribute(pxr::TfToken("physics:centerOfMass")).Get(&centerOfMass);
        _prim.GetAttribute(pxr::TfToken("physics:diagonalInertia")).Get(&diagonalInertia);

        if (mass < 0.0001)
        {
          mass = 1.0;
        }
        massMatrix.SetMass(mass);
        massMatrix.SetDiagonalMoments(
          ignition::math::Vector3d(diagonalInertia[0], diagonalInertia[1], diagonalInertia[2]));

        link->inertial->SetPose(ignition::math::Pose3d(
            ignition::math::Vector3d(
              centerOfMass[0], centerOfMass[1], centerOfMass[2]),
            ignition::math::Quaterniond(1.0, 0, 0, 0)));

        link->inertial->SetMassMatrix(massMatrix);
      }

      std::tuple<pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf, bool, bool, bool> transformsTuple = ParseTransform
        <pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf>(_prim);

      pxr::GfVec3f scale = std::get<0>(transformsTuple);
      pxr::GfVec3f translate = std::get<1>(transformsTuple);
      pxr::GfQuatf rotation_quad = std::get<2>(transformsTuple);

      bool isScale = std::get<3>(transformsTuple);;
      bool isTranslate = std::get<4>(transformsTuple);;
      bool isRotation = std::get<5>(transformsTuple);;

      if (isScale)
      {
        link->scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
      }
      if (isTranslate)
      {
        link->pose.Pos().X() = translate[0] * _metersPerUnit;
        link->pose.Pos().Y() = translate[1] * _metersPerUnit;
        link->pose.Pos().Z() = translate[2] * _metersPerUnit;
      }
      if (isRotation)
      {
        link->pose.Rot().X() = rotation_quad.GetImaginary()[0];
        link->pose.Rot().Y() = rotation_quad.GetImaginary()[1];
        link->pose.Rot().Z() = rotation_quad.GetImaginary()[2];
        link->pose.Rot().W() = rotation_quad.GetReal();
      }
    }

    sdf::Geometry geom;

    // if (!_prim.HasAPI<pxr::UsdPhysicsCollisionAPI>() &&
    //     (_prim.IsA<pxr::UsdGeomSphere>() ||
    //     _prim.IsA<pxr::UsdGeomCylinder>() ||
    //     _prim.IsA<pxr::UsdGeomCube>() ||
    //     _prim.IsA<pxr::UsdGeomMesh>()))
    // {
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

      std::tuple<pxr::GfVec3d, pxr::GfVec3d, pxr::GfQuatd, bool, bool, bool> transformsTuple = ParseTransform
        <pxr::GfVec3d, pxr::GfVec3d, pxr::GfQuatd>(_prim);

      pxr::GfVec3d scale = std::get<0>(transformsTuple);
      pxr::GfVec3d translate = std::get<1>(transformsTuple);
      pxr::GfQuatd rotation_quad = std::get<2>(transformsTuple);

      bool isScale = std::get<3>(transformsTuple);;
      bool isTranslate = std::get<4>(transformsTuple);;
      bool isRotation = std::get<5>(transformsTuple);;

      if (isScale)
        link->scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
      if (isTranslate && isRotation)
      {
        vis->SetRawPose(
          ignition::math::Pose3d(
            ignition::math::Vector3d(
              translate[0] * _metersPerUnit,
              translate[1] * _metersPerUnit,
              translate[2] * _metersPerUnit),
            ignition::math::Quaterniond(
              rotation_quad.GetReal(),
              rotation_quad.GetImaginary()[0],
              rotation_quad.GetImaginary()[1],
              rotation_quad.GetImaginary()[2])));
      }

      sdf::Material material = ParseMaterial(_prim);

      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        sdferr << "this is a Sphere\n";
        double radius;

        auto variant_sphere = pxr::UsdGeomSphere(_prim);
        variant_sphere.GetRadiusAttr().Get(&radius);
        sdferr << "radius sphere" << radius << "\n";

        sdf::Sphere s;
        geom.SetType(sdf::GeometryType::SPHERE);
        s.SetRadius(radius * _metersPerUnit * link->scale.X());
        geom.SetSphereShape(s);
        vis->SetName("visual_sphere");
        vis->SetGeom(geom);
        vis->SetMaterial(material);
      }
      else if (_prim.IsA<pxr::UsdGeomCylinder>())
      {
        auto variant_cylinder = pxr::UsdGeomCylinder(_prim);
        double radius;
        double height;
        variant_cylinder.GetRadiusAttr().Get(&radius);
        variant_cylinder.GetHeightAttr().Get(&height);

        sdf::Cylinder c;
        geom.SetType(sdf::GeometryType::CYLINDER);
        c.SetRadius(radius * _metersPerUnit * link->scale.X());
        c.SetLength(height * _metersPerUnit * link->scale.X());
        geom.SetCylinderShape(c);
        vis->SetName("visual_cylinder");
        vis->SetGeom(geom);
        vis->SetMaterial(material);
      }
      else if (_prim.IsA<pxr::UsdGeomCube>())
      {
        sdferr << "this is a cube\n";

        double size;
        auto variant_cylinder = pxr::UsdGeomCube(_prim);
        variant_cylinder.GetSizeAttr().Get(&size);

        size = size * _metersPerUnit;

        sdf::Box box;
        geom.SetType(sdf::GeometryType::BOX);
        box.SetSize(ignition::math::Vector3d(
          size * link->scale.X(),
          size * link->scale.Y(),
          size * link->scale.Z()));
        geom.SetBoxShape(box);
        vis->SetName("visual_box");
        vis->SetGeom(geom);
        vis->SetMaterial(material);
      }
      else if (_prim.IsA<pxr::UsdGeomMesh>())
      {
        std::cerr << "/* UsdGeomMesh */" << '\n';
        ignition::common::Mesh mesh;
        ignition::common::SubMesh subMesh;
        subMesh.SetName("lol");
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

        // std::cerr << "faceVertexCounts.size() " << faceVertexCounts.size() << '\n';
        // std::cerr << "faceVertexIndices.size() " << faceVertexIndices.size() << '\n';

        unsigned int indexVertex = 0;
        for (unsigned int i = 0; i < faceVertexCounts.size(); ++i)
        {
          // std::cerr << "faceVertexCounts[i] " << faceVertexCounts[i] << '\n';
          if (faceVertexCounts[i] == 3)
          {
            unsigned int j = indexVertex;
            unsigned int indexVertexStop = indexVertex + 3;
            for (; j < indexVertexStop; ++j)
            {
              // std::cerr << "faceVertexIndices[i] " << faceVertexIndices[j] << '\n';
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
        }

        for (auto & textCoord: textCoords)
        {
          // std::cerr << "textCoord " << textCoord << '\n';

          subMesh.AddTexCoord(textCoord[0], textCoord[1]);
        }

        for (auto & point: points)
        {
          // std::cerr << "point " << point << '\n';

          subMesh.AddVertex(point[0], point[1], point[2]);
        }

        for (auto & normal: normals)
        {
          // std::cerr << "normals " << normal << '\n';

          subMesh.AddNormal(normal[0], normal[1], normal[2]);
        }
        mesh.AddSubMesh(subMesh);

        sdf::Mesh meshGeom;
        geom.SetType(sdf::GeometryType::MESH);
        meshGeom.SetScale(ignition::math::Vector3d(link->scale.X(), link->scale.Y(), link->scale.Z()));
        geom.SetMeshShape(meshGeom);
        vis->SetName("visual_mesh");
        vis->SetGeom(geom);
        vis->SetMaterial(material);

        std::vector<std::string> tokens = ignition::common::split(link->name, "/");
        std::string directoryMesh;
        if (tokens.size() > 1)
        {
          for (int i = 0; i < tokens.size() - 1; ++i)
          {
            if (i == 0)
              directoryMesh = ignition::common::joinPaths(tokens[0], tokens[i+1]);
            else
              directoryMesh = ignition::common::joinPaths(directoryMesh, tokens[i+1]);
          }
        }
        if (ignition::common::createDirectories(directoryMesh))
        {
          directoryMesh = ignition::common::joinPaths(directoryMesh, tokens[tokens.size()-1]);

          // Export with extension
          ignition::common::ColladaExporter exporter;
          exporter.Export(&mesh, directoryMesh, false);
        }
        meshGeom.SetFilePath(directoryMesh + "dae");

      }

      pxr::TfTokenVector schemas = _prim.GetAppliedSchemas();
      int isOnlyCollision = 0;
      for (auto & token : schemas)
      {
        std::cerr << "GetText " << token.GetText() << '\n';
        if (std::string(token.GetText()) == "PhysicsCollisionAPI" ||
            std::string(token.GetText()) == "PhysxCollisionAPI" )
        {
          isOnlyCollision++;
        }
      }

      std::cerr << "isOnlyCollision " << isOnlyCollision << '\n';
      if (isOnlyCollision != 2)
      {
        link->visual_array.push_back(vis);
      }
    }

    bool collisionEnabled = false;
    if (_prim.HasAPI<pxr::UsdPhysicsCollisionAPI>())
    {
      std::cerr << "UsdPhysicsCollisionAPI" << '\n';

      _prim.GetAttribute(pxr::TfToken("physics:collisionEnabled")).Get(&collisionEnabled);
    }

    if (collisionEnabled)
    {
      sdf::Collision col;

      // add _collision extension
      std::string collisionName = link->name + kCollisionExt;
      col.SetName(collisionName);
      col.SetGeom(geom);

      std::tuple<pxr::GfVec3f, pxr::GfVec3d, pxr::GfQuatd, bool, bool, bool> transformsTuple = ParseTransform
        <pxr::GfVec3f, pxr::GfVec3d, pxr::GfQuatd>(_prim);

      pxr::GfVec3f scale = std::get<0>(transformsTuple);
      pxr::GfVec3d translate = std::get<1>(transformsTuple);
      pxr::GfQuatd rotation_quad = std::get<2>(transformsTuple);

      bool isScale = std::get<3>(transformsTuple);;
      bool isTranslate = std::get<4>(transformsTuple);;
      bool isRotation = std::get<5>(transformsTuple);;

      if (isScale)
      {
      }
      if (isTranslate)
      {
        col.SetRawPose(
          ignition::math::Pose3d(
            ignition::math::Vector3d(
              translate[0] * _metersPerUnit,
              translate[1] * _metersPerUnit,
              translate[2] * _metersPerUnit),
            col.RawPose().Rot()));
      }
      if (isRotation)
      {
        col.SetRawPose(
          ignition::math::Pose3d(
            col.RawPose().Pos(),
            ignition::math::Quaterniond(
              rotation_quad.GetReal(),
              rotation_quad.GetImaginary()[0],
              rotation_quad.GetImaginary()[1],
              rotation_quad.GetImaginary()[2])));
      }

      std::shared_ptr<sdf::Collision> colPtr = std::make_shared<sdf::Collision>();
      colPtr->SetRawPose(col.RawPose());
      colPtr->SetGeom(geom);
      colPtr->SetName(col.Name());

      if (_prim.IsA<pxr::UsdGeomCube>())
      {
        if (isScale)
        {
          const sdf::Box * box = colPtr->Geom()->BoxShape();
          sdf::Box * boxEditable = const_cast<sdf::Box*>(box);
          ignition::math::Vector3d size = box->Size();
          boxEditable->SetSize(ignition::math::Vector3d(
            size.X() * scale[0],
            size.Y() * scale[1],
            size.Z() * scale[2]));
          // std::cerr << "scale cube " << col.scale.X() << " " << col.scale.Y() << " link->scale.Z() " << col.scale.Z() << '\n';
          // std::cerr << "dim cube " << box->dim.X() << " " << box->dim.Y() << " " << box->dim.Z() << '\n';
        }
      }

      link->collision_array.push_back(colPtr);
      // Collision (optional)
      // Assign the first collision to the .collision ptr, if it exists
      if (!link->collision_array.empty())
        link->collision = link->collision_array[0];

      // TODO (ahcorde)
      // // set link visual material
      // if (link->visual)
      // {
      //   if (!link->visual->material_name.empty())
      //   {
      //     if (model->getMaterial(link->visual->material_name))
      //     {
      //       link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
      //     }
      //     else
      //     {
      //       if (link->visual->material)
      //       {
      //
      //         model->materials_.insert(make_pair(link->visual->material.name,link->visual->material));
      //       }
      //       else
      //       {
      //         model.reset();
      //         sdferr << "error material\n";
      //
      //         return model;
      //       }
      //     }
      //   }
      // }
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
