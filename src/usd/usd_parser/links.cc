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

#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/cube.h>

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"

#include "sdf/Console.hh"

#include "ignition/common/Util.hh"

#include "pxr/usd/usdPhysics/massAPI.h"
#include "pxr/usd/usdPhysics/collisionAPI.h"

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

    std::vector<std::string> tokens = ignition::common::split(pxr::TfStringify(_prim.GetPath()), "/");
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
      link->inertial.reset(new Inertial());
      link->inertial->clear();

      if (_prim.HasAPI<pxr::UsdPhysicsMassAPI>())
      {
        std::cerr << "UsdPhysicsMassAPI" << '\n';
        float mass;
        pxr::GfVec3f center_of_mass;
        _prim.GetAttribute(pxr::TfToken("physics:mass")).Get(&mass);
        _prim.GetAttribute(pxr::TfToken("physics:centerOfMass")).Get(&center_of_mass);

        std::cerr << "mass " << mass << '\n';


        link->inertial->mass = mass;
        if (link->inertial->mass < 0.0001)
        {
          link->inertial->mass = 1.0;
        }
        link->inertial->origin.position = usd::Vector3(
          center_of_mass[0], center_of_mass[1], center_of_mass[2]);
        link->inertial->origin.rotation = usd::Rotation(0, 0, 0, 1.0);

        // TODO(ahcorde): Review
        link->inertial->ixx = 0.16666;
        link->inertial->ixy = 0.0;
        link->inertial->ixz = 0.0;
        link->inertial->iyy = 0.16666;
        link->inertial->iyz = 0.0;
        link->inertial->izz = 0.16666;

        if (link->inertial->mass < 0.0001)
        {
          link->inertial->mass = 1.0;
        }
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
          link->scale = Vector3(scale[0], scale[1], scale[2]);
          std::cerr << "scale "<< scale << '\n';
        }

        if (op == "xformOp:translate")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
          link->pose.position.x = translate[0] * _metersPerUnit;
          link->pose.position.y = translate[1] * _metersPerUnit;
          link->pose.position.z = translate[2] * _metersPerUnit;
          std::cerr << "translate " << translate << '\n';

        }
        if (op == "xformOp:orient")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
          link->pose.rotation.x = rotation_quad.GetImaginary()[0];
          link->pose.rotation.y = rotation_quad.GetImaginary()[1];
          link->pose.rotation.z = rotation_quad.GetImaginary()[2];
          link->pose.rotation.w = rotation_quad.GetReal();
          std::cerr << "rotation_quad " << rotation_quad << '\n';
        }

        if (op == "xformOp:transform")
        {
          pxr::GfMatrix4d transform;
          _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
          pxr::GfVec3d translateMatrix = transform.ExtractTranslation();
          pxr::GfQuatd rotation_quadMatrix = transform.ExtractRotationQuat();
          link->pose.position.x = translateMatrix[0] * _metersPerUnit;
          link->pose.position.y = translateMatrix[1] * _metersPerUnit;
          link->pose.position.z = translateMatrix[2] * _metersPerUnit;
          link->pose.rotation.x = rotation_quadMatrix.GetImaginary()[0];
          link->pose.rotation.y = rotation_quadMatrix.GetImaginary()[1];
          link->pose.rotation.z = rotation_quadMatrix.GetImaginary()[2];
          link->pose.rotation.w = rotation_quadMatrix.GetReal();
          std::cerr << "translate " << translateMatrix << '\n';
          std::cerr << "rotation_quad " << rotation_quadMatrix << '\n';
        }
      }
    }

    // pxr::GfMatrix4d transform;
    // bool resetsXformStack;
    // variant_geom.GetLocalTransformation(&transform, &resetsXformStack);
    //
    // pxr::GfVec3d translate = transform.ExtractTranslation();
    // pxr::GfRotation rotation = transform.ExtractRotation();
    // pxr::GfQuatd rotation_quad = transform.ExtractRotationQuat();

    // sdferr << "translate " << translate << "\n";
    // sdferr << "rotation " << rotation.GetAxis() << "\n";
    // sdferr << "rotation " << rotation.GetAngle() << "\n";
    // sdferr << "rotation_quad " << rotation_quad << "\n";
    // sdferr << "rotation_quad real " << rotation_quad.GetReal() << "\n";
    // sdferr << "rotation_quad imaginary " << rotation_quad.GetImaginary() << "\n";

    // pxr::VtVec3fArray extentArray;
    // _prim.GetAttribute(pxr::TfToken("extent")).Get(&extentArray);

    GeometrySharedPtr geom;

    if (_prim.IsA<pxr::UsdGeomSphere>() || _prim.IsA<pxr::UsdGeomCylinder>() || _prim.IsA<pxr::UsdGeomCube>())
    {
      VisualSharedPtr vis;
      vis.reset(new Visual());
      vis->clear();

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
          link->scale = Vector3(scale[0], scale[1], scale[2]);
        }

        if (op == "xformOp:translate")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
          link->pose.position.x = translate[0] * _metersPerUnit;
          link->pose.position.y = translate[1] * _metersPerUnit;
          link->pose.position.z = translate[2] * _metersPerUnit;
        }
        if (op == "xformOp:orient")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
          link->pose.rotation.x = rotation_quad.GetImaginary()[0];
          link->pose.rotation.y = rotation_quad.GetImaginary()[1];
          link->pose.rotation.z = rotation_quad.GetImaginary()[2];
          link->pose.rotation.w = rotation_quad.GetReal();
        }

        if (op == "xformOp:transform")
        {
          pxr::GfMatrix4d transform;
          _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
          translate = transform.ExtractTranslation();
          rotation_quad = transform.ExtractRotationQuat();
          vis->origin.position.x = translate[0] * _metersPerUnit;
          vis->origin.position.y = translate[1] * _metersPerUnit;
          vis->origin.position.z = translate[2] * _metersPerUnit;
          vis->origin.rotation.x = rotation_quad.GetImaginary()[0];
          vis->origin.rotation.y = rotation_quad.GetImaginary()[1];
          vis->origin.rotation.z = rotation_quad.GetImaginary()[2];
          vis->origin.rotation.w = rotation_quad.GetReal();
        }
      }

      MaterialSharedPtr material;

      material.reset(new Material());
      variant_geom.GetDisplayColorAttr().Get(&color);

      pxr::VtFloatArray displayOpacity;
      _prim.GetAttribute(pxr::TfToken("primvars:displayOpacity")).Get(&displayOpacity);

      variant_geom.GetDisplayColorAttr().Get(&color);
      material->color.r = color[0][2];
      material->color.g = color[0][1];
      material->color.b = color[0][0];
      if (displayOpacity.size() > 0)
        material->color.a = 1 - displayOpacity[0];

      std::cerr << "color " << color << '\n';
      std::cerr << "displayOpacity " << displayOpacity << '\n';

      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        sdferr << "this is a Sphere\n";
        double radius;

        auto variant_sphere = pxr::UsdGeomSphere(_prim);
        variant_sphere.GetRadiusAttr().Get(&radius);
        sdferr << "radius sphere" << radius << "\n";

        // material->color.a = displayOpacity[0];

        // variant_sphere.GetDisplayColorAttr().Set(color);

        Sphere *s = new Sphere();
        geom.reset(s);
        s->clear();
        s->type = Geometry::SPHERE;
        s->radius = radius * _metersPerUnit * link->scale.x;
        vis->name = "visual_sphere";
        vis->geometry = geom;
        vis->material = material;
        link->visual_array.push_back(vis);
      }
      else if (_prim.IsA<pxr::UsdGeomCylinder>())
      {
        auto variant_cylinder = pxr::UsdGeomCylinder(_prim);
        double radius;
        double height;
        variant_cylinder.GetRadiusAttr().Get(&radius);
        variant_cylinder.GetHeightAttr().Get(&height);

        Cylinder * s = new Cylinder();
        geom.reset(s);
        s->clear();
        s->type = Geometry::CYLINDER;
        s->radius = radius * _metersPerUnit * link->scale.x;
        s->length = height * _metersPerUnit * link->scale.x;
        vis->name = "visual_cylinder";
        vis->geometry = geom;
        vis->material = material;
        link->visual_array.push_back(vis);
      }
      else if (_prim.IsA<pxr::UsdGeomCube>())
      {
        sdferr << "this is a cube\n";

        double size;
        auto variant_cylinder = pxr::UsdGeomCube(_prim);
        variant_cylinder.GetSizeAttr().Get(&size);

        size = size * _metersPerUnit;

        Box * box = new Box();
        geom.reset(box);
        box->clear();
        box->type = Geometry::BOX;
        box->dim = Vector3(size * link->scale.x, size * link->scale.y, size * link->scale.z);
        vis->name = "visual_box";
        vis->geometry = geom;
        vis->material = material;
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
      CollisionSharedPtr col;
      col.reset(new Collision());

      // add _collision extension
      std::string collisionName = link->name + kCollisionExt;
      col->name = collisionName;
      col->geometry = geom;

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
        std::cerr << "collision xformOpOrder " << op << '\n';
        std::string s = op;
        if (op == "xformOp:scale")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:scale")).Get(&scale);
        }

        if (op == "xformOp:translate")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&translate);
          col->origin.position.x = translate[0] * _metersPerUnit;
          col->origin.position.y = translate[1] * _metersPerUnit;
          col->origin.position.z = translate[2] * _metersPerUnit;
        }
        if (op == "xformOp:orient")
        {
          _prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&rotation_quad);
          col->origin.rotation.x = rotation_quad.GetImaginary()[0];
          col->origin.rotation.y = rotation_quad.GetImaginary()[1];
          col->origin.rotation.z = rotation_quad.GetImaginary()[2];
          col->origin.rotation.w = rotation_quad.GetReal();
        }

        if (op == "xformOp:transform")
        {
          pxr::GfMatrix4d transform;
          _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
          translate = transform.ExtractTranslation();
          rotation_quad = transform.ExtractRotationQuat();
          col->origin.position.x = translate[0] * _metersPerUnit;
          col->origin.position.y = translate[1] * _metersPerUnit;
          col->origin.position.z = translate[2] * _metersPerUnit;
          col->origin.rotation.x = rotation_quad.GetImaginary()[0];
          col->origin.rotation.y = rotation_quad.GetImaginary()[1];
          col->origin.rotation.z = rotation_quad.GetImaginary()[2];
          col->origin.rotation.w = rotation_quad.GetReal();
        }
      }

      link->collision_array.push_back(col);
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
      //         model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
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
