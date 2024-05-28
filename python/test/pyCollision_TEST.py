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
from gz_test_deps.math import Pose3d
from gz_test_deps.sdformat import (Box, Collision, Cone, Contact, Cylinder, Error,
                                   Geometry, Plane, Surface, Sphere,
                                   SDFErrorsException)
import gz_test_deps.sdformat as sdf
import unittest
import math

class CollisionTEST(unittest.TestCase):

    def test_default_construction(self):
        collision = Collision()
        self.assertTrue(not collision.name())

        collision.set_name("test_collison")
        self.assertEqual(collision.name(), "test_collison")

        self.assertEqual(Pose3d.ZERO, collision.raw_pose())
        self.assertTrue(not collision.pose_relative_to())

        semanticPose = collision.semantic_pose()
        self.assertEqual(collision.raw_pose(), semanticPose.raw_pose())
        self.assertTrue(not semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        collision.set_raw_pose(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi))
        self.assertEqual(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi),
                collision.raw_pose())

        collision.set_pose_relative_to("link")
        self.assertEqual("link", collision.pose_relative_to())

        semanticPose = collision.semantic_pose()
        self.assertEqual(collision.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("link", semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        self.assertNotEqual(None, collision.geometry())
        self.assertEqual(sdf.GeometryType.EMPTY, collision.geometry().type())
        self.assertEqual(None, collision.geometry().box_shape())
        self.assertEqual(None, collision.geometry().cone_shape())
        self.assertEqual(None, collision.geometry().cylinder_shape())
        self.assertEqual(None, collision.geometry().plane_shape())
        self.assertEqual(None, collision.geometry().sphere_shape())

        self.assertNotEqual(None, collision.surface())
        self.assertNotEqual(None, collision.surface().contact())


    def test_assignment(self):
        collision = Collision()
        collision.set_raw_pose(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi))

        collision2 = collision
        self.assertEqual(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi),
                collision2.raw_pose())


    def test_deepcopy(self):
        collision = Collision()
        collision.set_raw_pose(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi))

        collision2 = copy.deepcopy(collision)
        self.assertEqual(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi),
            collision2.raw_pose())


    def test_deepcopy_after_move(self):
        collision1 = Collision()
        collision1.set_raw_pose(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi))

        collision2 = Collision()
        collision2.set_raw_pose(Pose3d(-20, -30, -40, math.pi, math.pi, math.pi))

        # This is similar to what swap
        tmp = copy.deepcopy(collision1)
        collision1 = collision2
        collision2 = tmp

        self.assertEqual(Pose3d(-20, -30, -40, math.pi, math.pi, math.pi),
                collision1.raw_pose())
        self.assertEqual(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi),
                collision2.raw_pose())


    def test_set_geometry(self):
        collision = Collision()
        self.assertTrue(not collision.name())

        geometry = Geometry()
        geometry.set_type(sdf.GeometryType.BOX)

        collision.set_geometry(geometry)

        self.assertNotEqual(None, collision.geometry())
        self.assertEqual(sdf.GeometryType.BOX, collision.geometry().type())


    def test_set_surface(self):
        collision = Collision()

        surface = Surface()
        self.assertNotEqual(None, surface.contact())
        contact = Contact()
        contact.set_collide_bitmask(0x2)
        surface.set_contact(contact)

        collision.set_surface(surface)

        self.assertNotEqual(None, collision.surface())
        self.assertNotEqual(None, collision.surface().contact())
        self.assertEqual(collision.surface().contact().collide_bitmask(), 0x2)


if __name__ == '__main__':
    unittest.main()
