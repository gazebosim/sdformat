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

#include "pyNoise.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Noise.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineNoise(pybind11::object module)
{
  pybind11::class_<sdf::Noise> geometryModule(module, "Noise");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Noise>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("type", &sdf::Noise::Type,
         "Get the type of noise.")
    .def("set_type", &sdf::Noise::SetType,
         "Set the type of noise.")
    .def("mean", &sdf::Noise::Mean,
         "Get the mean of the Gaussian distribution "
         "from which noise values are drawn. This is applicable to "
         "\"gaussian*\" noise types.")
    .def("set_mean", &sdf::Noise::SetMean,
         "Set the mean of the Gaussian distribution "
         "from which noise values are drawn. This is applicable to "
         "\"gaussian*\" noise types.")
    .def("std_dev", &sdf::Noise::StdDev,
         "Get the StdDev of the Gaussian distribution "
         "from which noise values are drawn. This is applicable to "
         "\"gaussian*\" noise types.")
    .def("set_std_dev", &sdf::Noise::SetStdDev,
         "Set the StdDev of the Gaussian distribution "
         "from which noise values are drawn. This is applicable to "
         "\"gaussian*\" noise types.")
    .def("bias_mean", &sdf::Noise::BiasMean,
         "Get the mean of the Gaussian distribution "
         "from which bias values are drawn. This is applicable to \"gaussian*\""
         "noise types.")
    .def("set_bias_mean", &sdf::Noise::SetBiasMean,
         "Set the mean of the Gaussian distribution "
         "from which bias values are drawn. This is applicable to \"gaussian*\""
         "noise types.")
    .def("bias_std_dev", &sdf::Noise::BiasStdDev,
         "Get the standard deviation of the Gaussian distribution "
         "from which bias values are drawn. This is applicable to \"gaussian*\""
         "noise types.")
    .def("set_bias_std_dev", &sdf::Noise::SetBiasStdDev,
         "Set the standard deviation of the Gaussian distribution "
         "from which bias values are drawn. This is applicable to \"gaussian*\""
         "noise types.")
    .def("precision", &sdf::Noise::Precision,
         "For type \"gaussian_quantized\", get the precision of output "
         "signals. A value of zero implies infinite precision / no "
         "quantization.")
    .def("set_precision", &sdf::Noise::SetPrecision,
         "For type \"gaussian_quantized\", set the precision of output "
         "signals. A value of zero implies infinite precision / no "
         "quantization.")
    .def("dynamic_bias_std_dev", &sdf::Noise::DynamicBiasStdDev,
         "For type \"gaussian*\", get the standard deviation of the noise "
         "used to drive a process to model slow variations in a sensor bias.")
    .def("set_dynamic_bias_std_dev", &sdf::Noise::SetDynamicBiasStdDev,
         "For type \"gaussian*\", set the standard deviation of the noise "
         "used to drive a process to model slow variations in a sensor bias.")
    .def("dynamic_bias_correlation_time",
         &sdf::Noise::DynamicBiasCorrelationTime,
         "For type \"gaussian*\", get the correlation time of the noise "
         "used to drive a process to model slow variations in a sensor bias.")
    .def("set_dynamic_bias_correlation_time",
         &sdf::Noise::SetDynamicBiasCorrelationTime,
         "For type \"gaussian*\", set the correlation time in seconds of "
         "the noise used to drive a process to model slow variations in a "
         "sensor bias.A typical value, when used, would be on the order of "
         "3600 seconds (1 hour).")
    .def("__copy__", [](const sdf::Noise &self) {
      return sdf::Noise(self);
    })
    .def("__deepcopy__", [](const sdf::Noise &self, pybind11::dict) {
      return sdf::Noise(self);
    }, "memo"_a);

    pybind11::enum_<sdf::NoiseType>(geometryModule, "NoiseType")
      .value("NONE", sdf::NoiseType::NONE)
      .value("GAUSSIAN", sdf::NoiseType::GAUSSIAN)
      .value("GAUSSIAN_QUANTIZED", sdf::NoiseType::GAUSSIAN_QUANTIZED);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
