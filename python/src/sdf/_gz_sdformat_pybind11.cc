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
#include <gz/math/config.hh>

#include "pyAirFlow.hh"
#include "pyAirPressure.hh"
#include "pyAirSpeed.hh"
#include "pyAltimeter.hh"
#include "pyAtmosphere.hh"
#include "pyBox.hh"
#include "pyCamera.hh"
#include "pyCapsule.hh"
#include "pyCollision.hh"
#include "pyCylinder.hh"
#include "pyEllipsoid.hh"
#include "pyError.hh"
#include "pyExceptions.hh"
#include "pyForceTorque.hh"
#include "pyFrame.hh"
#include "pyGeometry.hh"
#include "pyGui.hh"
#include "pyHeightmap.hh"
#include "pyIMU.hh"
#include "pyJoint.hh"
#include "pyJointAxis.hh"
#include "pyLidar.hh"
#include "pyLight.hh"
#include "pyLink.hh"
#include "pyMagnetometer.hh"
#include "pyMaterial.hh"
#include "pyMesh.hh"
#include "pyModel.hh"
#include "pyNavSat.hh"
#include "pyNoise.hh"
#include "pyParserConfig.hh"
#include "pyParticleEmitter.hh"
#include "pyPbr.hh"
#include "pyPhysics.hh"
#include "pyPlane.hh"
#include "pyPlugin.hh"
#include "pyPolyline.hh"
#include "pyProjector.hh"
#include "pyRoot.hh"
#include "pyScene.hh"
#include "pySemanticPose.hh"
#include "pySensor.hh"
#include "pySky.hh"
#include "pySphere.hh"
#include "pySurface.hh"
#include "pyVisual.hh"
#include "pyWorld.hh"

PYBIND11_MODULE(BINDINGS_MODULE_NAME, m) {
  m.doc() = "sdformat Python Library.";

  // Import the gz.math library to automatically add the type conversions
  // this module requires to pass mathematical types to python code.
  std::string gzMathModule =
      std::string("gz.math") + std::to_string(GZ_MATH_MAJOR_VERSION);
  pybind11::module::import(gzMathModule.c_str());

  sdf::python::defineAirFlow(m);
  sdf::python::defineAirPressure(m);
  sdf::python::defineAirSpeed(m);
  sdf::python::defineAltimeter(m);
  sdf::python::defineAtmosphere(m);
  sdf::python::defineBox(m);
  sdf::python::defineBulletFriction(m);
  sdf::python::defineCamera(m);
  sdf::python::defineCapsule(m);
  sdf::python::defineCollision(m);
  sdf::python::defineContact(m);
  sdf::python::defineCylinder(m);
  sdf::python::defineEllipsoid(m);
  sdf::python::defineError(m);
  sdf::python::defineForceTorque(m);
  sdf::python::defineFrame(m);
  sdf::python::defineFriction(m);
  sdf::python::defineGeometry(m);
  sdf::python::defineGui(m);
  sdf::python::defineHeightmap(m);
  sdf::python::defineHeightmapBlend(m);
  sdf::python::defineHeightmapTexture(m);
  sdf::python::defineIMU(m);
  sdf::python::defineJoint(m);
  sdf::python::defineJointAxis(m);
  sdf::python::defineLidar(m);
  sdf::python::defineLight(m);
  sdf::python::defineLink(m);
  sdf::python::defineMagnetometer(m);
  sdf::python::defineMaterial(m);
  sdf::python::defineMesh(m);
  sdf::python::defineModel(m);
  sdf::python::defineNavSat(m);
  sdf::python::defineNoise(m);
  sdf::python::defineODE(m);
  sdf::python::defineParserConfig(m);
  sdf::python::defineParticleEmitter(m);
  sdf::python::definePbr(m);
  sdf::python::definePbrWorkflow(m);
  sdf::python::definePhysics(m);
  sdf::python::definePlane(m);
  sdf::python::definePlugin(m);
  sdf::python::definePolyline(m);
  sdf::python::defineProjector(m);
  sdf::python::defineRoot(m);
  sdf::python::defineScene(m);
  sdf::python::defineSemanticPose(m);
  sdf::python::defineSensor(m);
  sdf::python::defineSky(m);
  sdf::python::defineSphere(m);
  sdf::python::defineSurface(m);
  sdf::python::defineTorsional(m);
  sdf::python::defineVisual(m);
  sdf::python::defineWorld(m);

  m.attr("SDF_VERSION") = SDF_VERSION;
  m.attr("SDF_PROTOCOL_VERSION") = SDF_PROTOCOL_VERSION;

  static pybind11::exception<sdf::python::PySDFErrorsException>
      sdfErrorsException(m, "SDFErrorsException", PyExc_RuntimeError);

  pybind11::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) {
        std::rethrow_exception(p);
      }
    } catch (const sdf::python::PySDFErrorsException &e) {
      sdfErrorsException.attr("errors") = pybind11::cast(e.Errors());
      // This has to be called last since it's the call that sets
      // PyErr_SetString.
#if PYBIND11_VERSION_HEX >= 0x020C0000
      pybind11::set_error(sdfErrorsException, e.what());
#else
      sdfErrorsException(e.what());
#endif
    }
  });

}
