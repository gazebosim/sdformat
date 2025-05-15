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
from gz.math import Inertiald, MassMatrix3d, Pose3d, Vector3d
from sdformat import (Box, Collision, Cone, Contact, Cylinder, Error,
                                   Geometry, ParserConfig, Plane, Root, Surface, Sphere,
                                   SDFErrorsException)
import sdformat as sdf
import unittest
import math

class CollisionTEST(unittest.TestCase):

    def test_default_construction(self):
        collision = Collision()
        self.assertTrue(not collision.name())
        self.assertEqual(collision.density(), 1000.0)

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

        collision.set_density(1240.0)
        self.assertAlmostEqual(collision.density(), 1240.0)

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


    def test_incorrect_box_collision_calculate_inertial(self):
        collision = Collision()
        self.assertAlmostEqual(1000.0, collision.density())

        # sdf::ElementPtr sdf(new sdf::Element())
        # collision.Load(sdf)

        collisionInertial = Inertiald()
        sdfParserConfig = ParserConfig()
        geom = Geometry()
        box = Box()

        # Invalid Inertial test
        box.set_size(Vector3d(-1, 1, 0))
        geom.set_type(sdf.GeometryType.BOX)
        geom.set_box_shape(box)
        collision.set_geometry(geom)

        errors = []

        collision.calculate_inertial(errors, collisionInertial, sdfParserConfig)
        self.assertFalse(len(errors))


    def test_correct_box_collision_calculate_inertial(self):
        sdf = "<?xml version=\"1.0\"?>" + \
        " <sdf version=\"1.11\">" + \
        "   <model name='shapes'>" + \
        "     <link name='link'>" + \
        "       <inertial auto='true' />" + \
        "       <collision name='box_col'>" + \
        "         <density>1240.0</density>" + \
        "         <geometry>" + \
        "           <box>" + \
        "             <size>2 2 2</size>" + \
        "           </box>" + \
        "         </geometry>" + \
        "       </collision>" + \
        "     </link>" + \
        "   </model>" + \
        " </sdf>"

        root = Root()
        sdfParserConfig = ParserConfig()
        errors = root.load_sdf_string(sdf, sdfParserConfig)
        self.assertEqual(None, errors)

        model = root.model()
        link = model.link_by_index(0)
        collision = link.collision_by_index(0)

        inertialErr = []
        root.resolve_auto_inertials(inertialErr, sdfParserConfig)

        l = 2.0
        w = 2.0
        h = 2.0

        expectedMass = l * w * h * collision.density()
        ixx = (1.0 / 12.0) * expectedMass * (w * w + h * h)
        iyy = (1.0 / 12.0) * expectedMass * (l * l + h * h)
        izz = (1.0 / 12.0) * expectedMass * (l * l + w * w)

        expectedMassMat = MassMatrix3d(expectedMass, Vector3d(ixx, iyy, izz), Vector3d.ZERO)

        expectedInertial = Inertiald()
        expectedInertial.set_mass_matrix(expectedMassMat)
        expectedInertial.set_pose(Pose3d.ZERO)

        self.assertEqual(len(inertialErr), 0)
        self.assertAlmostEqual(1240.0, collision.density())
        self.assertAlmostEqual(expectedMass, link.inertial().mass_matrix().mass())
        self.assertEqual(expectedInertial.mass_matrix(), link.inertial().mass_matrix())
        self.assertEqual(expectedInertial.pose(), link.inertial().pose())

    def test_calculate_inertial_pose_not_relative_to_link(self):

        sdf = "<?xml version=\"1.0\"?>" + \
        " <sdf version=\"1.11\">" + \
        "   <model name='shapes'>" + \
        "     <frame name='arbitrary_frame'>" + \
        "       <pose>0 0 1 0 0 0</pose>" + \
        "     </frame>" + \
        "     <link name='link'>" + \
        "       <inertial auto='true' />" + \
        "       <collision name='box_col'>" + \
        "         <pose relative_to='arbitrary_frame'>0 0 -1 0 0 0</pose>" + \
        "         <density>1240.0</density>" + \
        "         <geometry>" + \
        "           <box>" + \
        "             <size>2 2 2</size>" + \
        "           </box>" + \
        "         </geometry>" + \
        "       </collision>" + \
        "     </link>" + \
        "   </model>" + \
        " </sdf>"

        root = Root()
        sdfParserConfig = ParserConfig()
        errors = root.load_sdf_string(sdf, sdfParserConfig)
        self.assertEqual(errors, None)
        # self.assertNotEqual(None, root.Element())

        model = root.model()
        link = model.link_by_index(0)
        collision = link.collision_by_index(0)

        inertialErr = []
        root.resolve_auto_inertials(inertialErr, sdfParserConfig)

        l = 2.0
        w = 2.0
        h = 2.0

        expectedMass = l * w * h * collision.density()
        ixx = (1.0 / 12.0) * expectedMass * (w * w + h * h)
        iyy = (1.0 / 12.0) * expectedMass * (l * l + h * h)
        izz = (1.0 / 12.0) * expectedMass * (l * l + w * w)

        expectedMassMat = MassMatrix3d(expectedMass, Vector3d(ixx, iyy, izz), Vector3d.ZERO)

        expectedInertial = Inertiald()
        expectedInertial.set_mass_matrix(expectedMassMat)
        expectedInertial.set_pose(Pose3d.ZERO)

        self.assertEqual(len(inertialErr), 0)
        self.assertAlmostEqual(1240.0, collision.density())
        self.assertAlmostEqual(expectedMass, link.inertial().mass_matrix().mass())
        self.assertEqual(expectedInertial.mass_matrix(), link.inertial().mass_matrix())
        self.assertEqual(expectedInertial.pose(), link.inertial().pose())

if __name__ == '__main__':
    unittest.main()
