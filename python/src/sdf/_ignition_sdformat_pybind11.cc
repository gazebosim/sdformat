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
 */

#include <pybind11/pybind11.h>

#include "pyBox.hh"
#include "pyCapsule.hh"
#include "pyCylinder.hh"
#include "pyEllipsoid.hh"
#include "pyError.hh"
#include "pyMesh.hh"
#include "pyParserConfig.hh"
#include "pyPlane.hh"
#include "pySphere.hh"
#include "pySurface.hh"

PYBIND11_MODULE(sdformat, m) {
  m.doc() = "sdformat Python Library.";

  sdf::python::defineBox(m);
  sdf::python::defineCapsule(m);
  sdf::python::defineContact(m);
  sdf::python::defineCylinder(m);
  sdf::python::defineEllipsoid(m);
  sdf::python::defineError(m);
  sdf::python::defineMesh(m);
  sdf::python::defineParserConfig(m);
  sdf::python::definePlane(m);
  sdf::python::defineSphere(m);
  sdf::python::defineSurface(m);
}
