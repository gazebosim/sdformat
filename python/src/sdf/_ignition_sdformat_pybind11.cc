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
#include <pybind11/stl.h>

#include "pyBox.hh"
#include "pyCapsule.hh"
#include "pyCollision.hh"
#include "pyCylinder.hh"
#include "pyEllipsoid.hh"
#include "pyError.hh"
#include "pyExceptions.hh"
#include "pyFrame.hh"
#include "pyGeometry.hh"
#include "pyJoint.hh"
#include "pyJointAxis.hh"
#include "pyLink.hh"
#include "pyMaterial.hh"
#include "pyMesh.hh"
#include "pyModel.hh"
#include "pyNoise.hh"
#include "pyParserConfig.hh"
#include "pyPlane.hh"
#include "pySemanticPose.hh"
#include "pySphere.hh"
#include "pySurface.hh"
#include "pyVisual.hh"
#include "pyWorld.hh"

static PyObject *PySDFErrorsException;

PYBIND11_MODULE(sdformat, m) {
  m.doc() = "sdformat Python Library.";

  PySDFErrorsException = PyErr_NewException("sdformat.SDFErrorsException", NULL, NULL);
  if (PySDFErrorsException) {
    PyTypeObject *as_type = reinterpret_cast<PyTypeObject *>(PySDFErrorsException);
    as_type->tp_str = sdf::python::SDFErrorsException_tp_str;
    PyObject *descr = PyDescr_NewGetSet(as_type, sdf::python::SDFErrorsException_getsetters);
    auto dict = pybind11::reinterpret_borrow<pybind11::dict>(as_type->tp_dict);
    dict[pybind11::handle(PyDescr_NAME(descr))] = pybind11::handle(descr);

    Py_XINCREF(PySDFErrorsException);
    m.add_object("SDFErrorsException", pybind11::handle(PySDFErrorsException));
  }

  pybind11::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) {
        std::rethrow_exception(p);
      }
    } catch (sdf::python::SDFErrorsException &e) {
      pybind11::tuple args(2);
      args[0] = e.getErrorString();
      args[1] = e.GetErrors();
      PyErr_SetObject(PySDFErrorsException, args.ptr());
    }
  });

  sdf::python::defineBox(m);
  sdf::python::defineCapsule(m);
  sdf::python::defineCollision(m);
  sdf::python::defineContact(m);
  sdf::python::defineCylinder(m);
  sdf::python::defineEllipsoid(m);
  sdf::python::defineError(m);
  sdf::python::defineExceptions(m);
  sdf::python::defineFrame(m);
  sdf::python::defineGeometry(m);
  sdf::python::defineJoint(m);
  sdf::python::defineJointAxis(m);
  sdf::python::defineLink(m);
  sdf::python::defineMaterial(m);
  sdf::python::defineMesh(m);
  sdf::python::defineModel(m);
  sdf::python::defineNoise(m);
  sdf::python::defineParserConfig(m);
  sdf::python::definePlane(m);
  sdf::python::defineSemanticPose(m);
  sdf::python::defineSphere(m);
  sdf::python::defineSurface(m);
  sdf::python::defineVisual(m);
  sdf::python::defineWorld(m);
}
