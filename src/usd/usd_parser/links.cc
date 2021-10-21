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

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"

#include "sdf/Console.hh"


namespace usd
{
  const char kCollisionExt[] = "_collision";

  LinkSharedPtr ParseLinks(const pxr::UsdPrim &_prim)
  {
    LinkSharedPtr link = nullptr;

    pxr::VtArray<pxr::GfVec3f> color {{1, 0, 0}};

    if (_prim.IsA<pxr::UsdGeomSphere>() || _prim.IsA<pxr::UsdGeomCylinder>())
    {
      float mass;
      pxr::GfVec3f center_of_mass;
      bool collisionEnabled;
      _prim.GetAttribute(pxr::TfToken("physics:mass")).Get(&mass);
      _prim.GetAttribute(pxr::TfToken("physics:centerOfMass")).Get(&center_of_mass);
      _prim.GetAttribute(pxr::TfToken("physics:collisionEnabled")).Get(&collisionEnabled);

      link.reset(new Link);
      link->clear();

      link->name = pxr::TfStringify(_prim.GetPath());
      link->inertial.reset(new Inertial());

      link->inertial->clear();
      link->inertial->mass = mass;
      link->inertial->origin.position = usd::Vector3(
        center_of_mass[0], center_of_mass[1], center_of_mass[2]);
      link->inertial->origin.rotation = usd::Rotation(0, 0, 0, 1.0);
      link->inertial->ixx = 0.16666;
      link->inertial->ixy = 0.0;
      link->inertial->ixz = 0.0;
      link->inertial->iyy = 0.16666;
      link->inertial->iyz = 0.0;
      link->inertial->izz = 0.16666;

      auto variant_geom = pxr::UsdGeomGprim(_prim);

      auto transforms = variant_geom.GetXformOpOrderAttr();

      if (!transforms)
      {
        sdferr << "No transforms available!\n";
      }

      pxr::GfMatrix4d transform;
      bool resetsXformStack;
      variant_geom.GetLocalTransformation(&transform, &resetsXformStack);

      pxr::GfVec3d translate = transform.ExtractTranslation();
      pxr::GfRotation rotation = transform.ExtractRotation();
      pxr::GfQuatd rotation_quad = transform.ExtractRotationQuat();
      // sdferr << "translate " << translate << "\n";
      // sdferr << "rotation " << rotation.GetAxis() << "\n";
      // sdferr << "rotation " << rotation.GetAngle() << "\n";
      // sdferr << "rotation_quad " << rotation_quad << "\n";
      // sdferr << "rotation_quad real " << rotation_quad.GetReal() << "\n";
      // sdferr << "rotation_quad imaginary " << rotation_quad.GetImaginary() << "\n";

      VisualSharedPtr vis;
      vis.reset(new Visual());
      vis->clear();

      GeometrySharedPtr geom;
      if (_prim.IsA<pxr::UsdGeomSphere>())
      {
        sdferr << "this is a Sphere\n";
        double radius;

        auto variant_sphere = pxr::UsdGeomSphere(_prim);
        variant_sphere.GetRadiusAttr().Get(&radius);
        sdferr << "radius sphere" << radius << "\n";
        variant_sphere.GetDisplayColorAttr().Get(&color);

        // variant_sphere.GetDisplayColorAttr().Set(color);

        Sphere *s = new Sphere();
        geom.reset(s);
        s->clear();
        s->type = Geometry::SPHERE;
        s->radius = radius;
        vis->name = "visual_sphere";
        vis->geometry = geom;
        link->visual_array.push_back(vis);
      }
      else if (_prim.IsA<pxr::UsdGeomCylinder>())
      {
        sdferr << "this is a cylinder\n";
        auto variant_cylinder = pxr::UsdGeomCylinder(_prim);
        double radius;
        double height;
        variant_cylinder.GetRadiusAttr().Get(&radius);
        variant_cylinder.GetHeightAttr().Get(&height);

        Cylinder *s = new Cylinder();
        geom.reset(s);
        s->clear();
        s->type = Geometry::CYLINDER;
        s->radius = radius;
        s->length = height;
        vis->name = "visual_cylinder";
        link->visual_array.push_back(vis);
      }

      if (collisionEnabled)
      {
        CollisionSharedPtr col;
        col.reset(new Collision());

        // add _collision extension
        std::string collisionName = link->name + kCollisionExt;
        col->name = collisionName;
        col->geometry = geom;

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
    }

    std::cerr << "color " << color[0] << " " << color[1] << " " << color[2] << '\n';

    // Visual (optional)
    // Assign the first visual to the .visual ptr, if it exists
    if (!link->visual_array.empty())
      link->visual = link->visual_array[0];

    return link;
  }
}
