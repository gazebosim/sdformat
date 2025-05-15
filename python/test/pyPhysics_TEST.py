# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from sdformat import Physics
import unittest


class PhysicsTEST(unittest.TestCase):


    def test_default_construction(self):
        physics = Physics()
        self.assertFalse(physics.name())

        physics.set_name("test_physics")
        self.assertEqual("test_physics", physics.name())

        self.assertFalse(physics.is_default())
        physics.set_default(True)
        self.assertTrue(physics.is_default())

        self.assertEqual("ode", physics.engine_type())
        physics.set_engine_type("bullet")
        self.assertEqual("bullet", physics.engine_type())

        self.assertAlmostEqual(0.001, physics.max_step_size())
        physics.set_max_step_size(1.234)
        self.assertAlmostEqual(1.234, physics.max_step_size())

        self.assertAlmostEqual(1.0, physics.real_time_factor())
        physics.set_real_time_factor(2.45)
        self.assertAlmostEqual(2.45, physics.real_time_factor())


    def test_copy_construction(self):
        physics = Physics()
        physics.set_name("test_physics")

        physics2 = Physics(physics)
        self.assertEqual("test_physics", physics2.name())


    def test_assignment_construction(self):
        physics = Physics()
        physics.set_name("test_physics")

        physics2 = physics
        self.assertEqual("test_physics", physics2.name())


    def test_deepcopy(self):
        physics = Physics()
        physics.set_name("test_physics")

        physics2 = copy.deepcopy(physics)
        self.assertEqual("test_physics", physics2.name())


if __name__ == '__main__':
    unittest.main()
