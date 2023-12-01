# Copyright (C) 2023 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from gz_test_deps.sdformat import AirFlow, Noise
import gz_test_deps.sdformat as sdf
import unittest

class AirFlowTEST(unittest.TestCase):


    def test_default_construction(self):
        airflow = AirFlow()
        defaultNoise = Noise()
        self.assertEqual(defaultNoise, airflow.direction_noise())
        self.assertEqual(defaultNoise, airflow.speed_noise())

    def test_set(self):
        air = AirFlow()
        defaultNoise = Noise()
        dir_noise = Noise()
        speed_noise = Noise()
        self.assertEqual(defaultNoise, air.speed_noise())
        self.assertEqual(defaultNoise, air.direction_noise())

        dir_noise.set_type(sdf.NoiseType.GAUSSIAN)
        dir_noise.set_mean(1.2)
        dir_noise.set_std_dev(2.3)
        dir_noise.set_bias_mean(4.5)
        dir_noise.set_bias_std_dev(6.7)
        dir_noise.set_precision(8.9)

        speed_noise.set_type(sdf.NoiseType.GAUSSIAN)
        speed_noise.set_mean(1.2)
        speed_noise.set_std_dev(2.3)
        speed_noise.set_bias_mean(4.5)
        speed_noise.set_bias_std_dev(6.7)
        speed_noise.set_precision(8.9)

        air.set_direction_noise(dir_noise)
        air.set_speed_noise(speed_noise)
        self.assertEqual(dir_noise, air.direction_noise())
        self.assertEqual(speed_noise, air.speed_noise())

        # Copy Constructor
        air2 = AirFlow(air)
        self.assertEqual(air, air2)

        # Copy operator
        air3 = air
        self.assertEqual(air, air3)

        air4 = AirFlow()
        self.assertNotEqual(air3, air4);


if __name__ == '__main__':
    unittest.main()
