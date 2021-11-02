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


namespace usd
{
  const char kCollisionExt[] = "_collision";

  LinkSharedPtr ParseLinks(const pxr::UsdPrim &_prim, LinkSharedPtr &link, const double _metersPerUnit)
  {
    std::cerr << "************ ADDED LINK ************" << '\n';

    pxr::VtArray<pxr::GfVec3f> color {{1, 0, 0}};

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
            ignition::math::Quaterniond(0, 0, 0, 1.0)));

        link->inertial->SetMassMatrix(massMatrix);
      }

      auto variant_geom = pxr::UsdGeomGprim(_prim);

      auto transforms = variant_geom.GetXformOpOrderAttr();

      if (!transforms)
      {
        sdferr << "No transforms available!\n";
      }

      pxr::GfVec3f scale(1, 1, 1);
      pxr::GfVec3f translate(0, 0, 0);
      pxr::GfQuatf rotation_quad(1, 0, 0, 0);

      pxr::VtTokenArray xformOpOrder;
      transforms.Get(&xformOpOrder);
      for (auto & op: xformOpOrder)
      {
        std::cerr << "xformOpOrder " << op << '\n';
        std::string s = op;
        if (op == "xformOp:scale")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:scale")).Get(&scale);
          link->scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
          std::cerr << "scale "<< scale << '\n';
        }

        if (op == "xformOp:translate")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
          link->pose.Pos().X() = translate[0] * _metersPerUnit;
          link->pose.Pos().Y() = translate[1] * _metersPerUnit;
          link->pose.Pos().Z() = translate[2] * _metersPerUnit;
          std::cerr << "translate " << translate << '\n';

        }
        if (op == "xformOp:orient")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
          link->pose.Rot().X() = rotation_quad.GetImaginary()[0];
          link->pose.Rot().Y() = rotation_quad.GetImaginary()[1];
          link->pose.Rot().Z() = rotation_quad.GetImaginary()[2];
          link->pose.Rot().W() = rotation_quad.GetReal();
          std::cerr << "rotation_quad " << rotation_quad << '\n';
        }

        if (op == "xformOp:transform")
        {
          pxr::GfMatrix4d transform;
          _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
          pxr::GfVec3d translateMatrix = transform.ExtractTranslation();
          pxr::GfQuatd rotation_quadMatrix = transform.ExtractRotationQuat();
          link->pose.Pos().X() = translateMatrix[0] * _metersPerUnit;
          link->pose.Pos().Y() = translateMatrix[1] * _metersPerUnit;
          link->pose.Pos().Z() = translateMatrix[2] * _metersPerUnit;
          link->pose.Rot().X() = rotation_quadMatrix.GetImaginary()[0];
          link->pose.Rot().Y() = rotation_quadMatrix.GetImaginary()[1];
          link->pose.Rot().Z() = rotation_quadMatrix.GetImaginary()[2];
          link->pose.Rot().W() = rotation_quadMatrix.GetReal();
          std::cerr << "translate " << translateMatrix << '\n';
          std::cerr << "rotation_quad " << rotation_quadMatrix << '\n';
        }
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

      pxr::GfVec3d scale(1, 1, 1);
      pxr::GfVec3d translate(0, 0, 0);
      pxr::GfQuatd rotation_quad(1, 0, 0, 0);

      pxr::VtTokenArray xformOpOrder;
      transforms.Get(&xformOpOrder);
      for (auto & op: xformOpOrder)
      {
        std::cerr << "xformOpOrder " << op << '\n';
        std::string s = op;
        if (op == "xformOp:scale")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:scale")).Get(&scale);
          link->scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
        }

        if (op == "xformOp:translate")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
          link->pose.Pos().X() = translate[0] * _metersPerUnit;
          link->pose.Pos().Y() = translate[1] * _metersPerUnit;
          link->pose.Pos().Z() = translate[2] * _metersPerUnit;
        }
        if (op == "xformOp:orient")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
          link->pose.Rot().X() = rotation_quad.GetImaginary()[0];
          link->pose.Rot().Y() = rotation_quad.GetImaginary()[1];
          link->pose.Rot().Z() = rotation_quad.GetImaginary()[2];
          link->pose.Rot().W() = rotation_quad.GetReal();
        }

        if (op == "xformOp:transform")
        {
          pxr::GfMatrix4d transform;
          _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
          translate = transform.ExtractTranslation();
          rotation_quad = transform.ExtractRotationQuat();
          vis->SetRawPose(
            ignition::math::Pose3d(
              ignition::math::Vector3d(
                translate[0] * _metersPerUnit,
                translate[1] * _metersPerUnit,
                translate[2] * _metersPerUnit),
              ignition::math::Quaterniond(
                rotation_quad.GetImaginary()[0],
                rotation_quad.GetImaginary()[1],
                rotation_quad.GetImaginary()[2],
                rotation_quad.GetReal())));
        }
      }

      sdf::Material material;

      variant_geom.GetDisplayColorAttr().Get(&color);

      pxr::VtFloatArray displayOpacity;
      _prim.GetAttribute(pxr::TfToken("primvars:displayOpacity")).Get(&displayOpacity);

      variant_geom.GetDisplayColorAttr().Get(&color);
      double alpha = 1.0;
      if (displayOpacity.size() > 0)
      {
        alpha = 1 - displayOpacity[0];
      }
      material.SetAmbient(
        ignition::math::Color(
          ignition::math::clamp(color[0][2] / 0.4, 0.0, 1.0),
          ignition::math::clamp(color[0][1] / 0.4, 0.0, 1.0),
          ignition::math::clamp(color[0][0] / 0.4, 0.0, 1.0),
          alpha));
      material.SetDiffuse(
        ignition::math::Color(
          ignition::math::clamp(color[0][2] / 0.8, 0.0, 1.0),
          ignition::math::clamp(color[0][1] / 0.8, 0.0, 1.0),
          ignition::math::clamp(color[0][0] / 0.8, 0.0, 1.0),
          alpha));

      std::cerr << "color " << color << '\n';
      std::cerr << "displayOpacity " << displayOpacity << '\n';

      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        sdferr << "this is a Sphere\n";
        double radius;

        auto variant_sphere = pxr::UsdGeomSphere(_prim);
        variant_sphere.GetRadiusAttr().Get(&radius);
        sdferr << "radius sphere" << radius << "\n";

        // material.color.a = displayOpacity[0];

        // variant_sphere.GetDisplayColorAttr().Set(color);

        sdf::Sphere s;
        geom.SetType(sdf::GeometryType::SPHERE);
        s.SetRadius(radius * _metersPerUnit * link->scale.X());
        geom.SetSphereShape(s);
        vis->SetName("visual_sphere");
        vis->SetGeom(geom);
        vis->SetMaterial(material);
        link->visual_array.push_back(vis);
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
        link->visual_array.push_back(vis);
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
        link->visual_array.push_back(vis);
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

        std::cerr << "faceVertexCounts.size() " << faceVertexCounts.size() << '\n';
        std::cerr << "faceVertexIndices.size() " << faceVertexIndices.size() << '\n';

        unsigned int indexVertex = 0;
        for (unsigned int i = 0; i < faceVertexCounts.size(); ++i)
        {
          std::cerr << "faceVertexCounts[i] " << faceVertexCounts[i] << '\n';
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
          std::cerr << "textCoord " << textCoord << '\n';

          subMesh.AddTexCoord(textCoord[0], textCoord[1]);
        }

        for (auto & point: points)
        {
          std::cerr << "point " << point << '\n';

          subMesh.AddVertex(point[0], point[1], point[2]);
        }

        for (auto & normal: normals)
        {
          std::cerr << "normals " << normal << '\n';

          subMesh.AddNormal(normal[0], normal[1], normal[2]);
        }
        mesh.AddSubMesh(subMesh);

        sdf::Mesh meshGeom;
        geom.SetType(sdf::GeometryType::MESH);
        meshGeom.SetScale(ignition::math::Vector3d(link->scale.X(), link->scale.Y(), link->scale.Z()));
        meshGeom.SetFilePath("/home/ahcorde/tmp/sdformat/build/salida.dae");
        geom.SetMeshShape(meshGeom);
        vis->SetName("visual_mesh");
        vis->SetGeom(geom);
        vis->SetMaterial(material);
        link->visual_array.push_back(vis);

        // Export with extension
        ignition::common::ColladaExporter exporter;
        exporter.Export(&mesh, "/home/ahcorde/tmp/sdformat/build/salida", false);
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

      auto variant_geom = pxr::UsdGeomGprim(_prim);

      auto transforms = variant_geom.GetXformOpOrderAttr();

      if (!transforms)
      {
        sdferr << "No transforms available!\n";
      }

      pxr::GfVec3f scale(1, 1, 1);
      pxr::GfVec3d translate(0, 0, 0);
      pxr::GfQuatd rotation_quad(1, 0, 0, 0);

      pxr::VtTokenArray xformOpOrder;
      transforms.Get(&xformOpOrder);
      for (auto & op: xformOpOrder)
      {
        std::cerr << "collision xformOpOrder " << op << '\n';
        std::string s = op;
        if (op == "xformOp:scale")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:scale")).Get(&scale);
          std::cerr << "col scale " << scale[0] << " " << scale[1] << " " << scale[2] << '\n';
          // col.scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
        }

        if (op == "xformOp:translate")
        {
          // FIX this
          pxr::GfVec3f translate(0, 0, 0);
          _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
          col.SetRawPose(
            ignition::math::Pose3d(
              ignition::math::Vector3d(
                translate[0] * _metersPerUnit,
                translate[1] * _metersPerUnit,
                translate[2] * _metersPerUnit),
              col.RawPose().Rot()));
        }
        if (op == "xformOp:orient")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
          col.SetRawPose(
            ignition::math::Pose3d(
              col.RawPose().Pos(),
              ignition::math::Quaterniond(
                rotation_quad.GetImaginary()[0],
                rotation_quad.GetImaginary()[1],
                rotation_quad.GetImaginary()[2],
                rotation_quad.GetReal())));
        }

        if (op == "xformOp:transform")
        {
          pxr::GfMatrix4d transform;
          _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
          translate = transform.ExtractTranslation();
          rotation_quad = transform.ExtractRotationQuat();
          col.SetRawPose(
            ignition::math::Pose3d(
              ignition::math::Vector3d(
                translate[0] * _metersPerUnit,
                translate[1] * _metersPerUnit,
                translate[2] * _metersPerUnit),
              ignition::math::Quaterniond(
                rotation_quad.GetImaginary()[0],
                rotation_quad.GetImaginary()[1],
                rotation_quad.GetImaginary()[2],
                rotation_quad.GetReal())));
        }
      }

      if (_prim.IsA<pxr::UsdGeomCube>())
      {
        const sdf::Box * box = col.Geom()->BoxShape();
        ignition::math::Vector3d size = box->Size();
        // box->SetSize(ignition::math::Vector3d(
        //   size.X() * col.scale.X(),
        //   size.Y() * col.scale.Y(),
        //   size.Z() * col.scale.Z()));
        // std::cerr << "scale cube " << col.scale.X() << " " << col.scale.Y() << " link->scale.Z() " << col.scale.Z() << '\n';
        // std::cerr << "dim cube " << box->dim.X() << " " << box->dim.Y() << " " << box->dim.Z() << '\n';
      }

      std::shared_ptr<sdf::Collision> colPtr = std::make_shared<sdf::Collision>();
      colPtr->SetRawPose(col.RawPose());
      colPtr->SetGeom(geom);
      colPtr->SetName(col.Name());
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

    // std::cerr << "color " << color[0] << " " << color[1] << " " << color[2] << '\n';

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
