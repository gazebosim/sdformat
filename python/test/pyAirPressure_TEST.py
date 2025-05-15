# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from sdformat import AirPressure, Noise
import sdformat as sdf
import unittest

class AtmosphereTEST(unittest.TestCase):


    def test_default_construction(self):
        air = AirPressure()
        defaultNoise = Noise()
        self.assertEqual(defaultNoise, air.pressure_noise())
        self.assertAlmostEqual(0.0, air.reference_altitude())

    def test_set(self):
        air = AirPressure()
        defaultNoise = Noise()
        noise = Noise()
        self.assertEqual(defaultNoise, air.pressure_noise())
        self.assertAlmostEqual(0.0, air.reference_altitude())

        noise.set_type(sdf.NoiseType.GAUSSIAN)
        noise.set_mean(1.2)
        noise.set_std_dev(2.3)
        noise.set_bias_mean(4.5)
        noise.set_bias_std_dev(6.7)
        noise.set_precision(8.9)

        air.set_pressure_noise(noise)
        self.assertEqual(noise, air.pressure_noise())

        air.set_reference_altitude(10.2)
        self.assertAlmostEqual(10.2, air.reference_altitude())

        # Copy Constructor
        air2 = AirPressure(air)
        self.assertEqual(air, air2)

        # Copy operator
        air3 = air
        self.assertEqual(air, air3)

        air4 = AirPressure()
        self.assertNotEqual(air3, air4);


if __name__ == '__main__':
    unittest.main()
