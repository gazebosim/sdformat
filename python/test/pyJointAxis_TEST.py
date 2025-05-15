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

import copy
from gz.math import Vector3d
from sdformat import JointAxis, MimicConstraint, Error, SDFErrorsException
import math
import unittest


class JointAxisTEST(unittest.TestCase):

    def test_default_construction(self):
        axis = JointAxis()
        self.assertEqual(Vector3d.UNIT_Z, axis.xyz())
        self.assertFalse(axis.xyz_expressed_in())
        self.assertAlmostEqual(0.0, axis.damping())
        self.assertAlmostEqual(0.0, axis.friction())
        self.assertAlmostEqual(0.0, axis.spring_reference())
        self.assertAlmostEqual(0.0, axis.spring_stiffness())
        self.assertAlmostEqual(-math.inf, axis.lower())
        self.assertAlmostEqual(math.inf, axis.upper())
        self.assertAlmostEqual(math.inf, axis.effort())
        self.assertAlmostEqual(math.inf, axis.max_velocity())
        self.assertAlmostEqual(1e8, axis.stiffness())
        self.assertAlmostEqual(1.0, axis.dissipation())

        axis.set_xyz(Vector3d(0, 1, 0))
        self.assertEqual(Vector3d.UNIT_Y, axis.xyz())

        axis.set_xyz_expressed_in("__model__")
        self.assertEqual("__model__", axis.xyz_expressed_in())

        # expect errors when trying to resolve axis without graph
        with self.assertRaises(SDFErrorsException):
            axis.resolve_xyz()

        axis.set_damping(0.2)
        self.assertAlmostEqual(0.2, axis.damping())

        axis.set_friction(1.3)
        self.assertAlmostEqual(1.3, axis.friction())

        axis.set_spring_reference(2.4)
        self.assertAlmostEqual(2.4, axis.spring_reference())

        axis.set_spring_stiffness(-1.2)
        self.assertAlmostEqual(-1.2, axis.spring_stiffness())

        axis.set_lower(-10.8)
        self.assertAlmostEqual(-10.8, axis.lower())

        axis.set_upper(123.4)
        self.assertAlmostEqual(123.4, axis.upper())

        axis.set_effort(3.2)
        self.assertAlmostEqual(3.2, axis.effort())

        axis.set_max_velocity(54.2)
        self.assertAlmostEqual(54.2, axis.max_velocity())

        axis.set_stiffness(1e2)
        self.assertAlmostEqual(1e2, axis.stiffness())

        axis.set_dissipation(1.5)
        self.assertAlmostEqual(1.5, axis.dissipation())

        mimic = MimicConstraint("test_joint", "axis", 5.0, 1.0, 2.0)

        self.assertEqual(None, axis.mimic())
        axis.set_mimic(mimic)
        self.assertNotEqual(None, axis.mimic())
        self.assertEqual("test_joint", axis.mimic().joint())
        self.assertEqual("axis", axis.mimic().axis())
        self.assertAlmostEqual(5.0, axis.mimic().multiplier())
        self.assertAlmostEqual(1.0, axis.mimic().offset())
        self.assertAlmostEqual(2.0, axis.mimic().reference())


    def test_copy_construction(self):
        jointAxis = JointAxis()
        self.assertEqual(0, len(jointAxis.set_xyz(Vector3d(0, 1, 0))))

        jointAxisCopy = JointAxis(jointAxis)
        self.assertEqual(jointAxis.xyz(), jointAxisCopy.xyz())


    def test_copy_construction(self):
        jointAxis = JointAxis()
        jointAxis.set_xyz(Vector3d(0, 1, 0))

        jointAxisCopy = copy.deepcopy(jointAxis)
        self.assertEqual(jointAxis.xyz(), jointAxisCopy.xyz())


    def test_zero_norm_vector_returns_error(self):
        axis = JointAxis()
        axis.set_xyz(Vector3d(1.0, 0, 0))

        with self.assertRaises(SDFErrorsException) as cm:
            axis.set_xyz(Vector3d.ZERO)
        errors = cm.exception.errors
        self.assertTrue(errors)
        self.assertEqual(errors[0].message(), "The norm of the xyz vector cannot be zero")


if __name__ == '__main__':
    unittest.main()
