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
from gz_test_deps.math import Pose3d, Inertiald, MassMatrix3d, Vector3d
from gz_test_deps.sdformat import (Collision, Light, Link, Projector, Sensor,
                                   Visual, SDFErrorsException)
import unittest
import math


class LinkTEST(unittest.TestCase):

    def test_default_construction(self):

        link = Link()
        self.assertTrue(not link.name())

        link.set_name("test_link")
        self.assertEqual("test_link", link.name())

        self.assertEqual(0, link.visual_count())
        self.assertEqual(None, link.visual_by_index(0))
        self.assertEqual(None, link.visual_by_index(1))
        self.assertFalse(link.visual_name_exists(""))
        self.assertFalse(link.visual_name_exists("default"))

        self.assertEqual(0, link.light_count())
        self.assertEqual(None, link.light_by_index(0))
        self.assertEqual(None, link.light_by_index(1))
        self.assertFalse(link.light_name_exists(""))
        self.assertFalse(link.light_name_exists("default"))
        self.assertEqual(None, link.light_by_name("no_such_light"))

        # self.assertEqual(0, link.particle_emitter_count())
        # self.assertEqual(None, link.ParticleEmitterByIndex(0))
        # self.assertEqual(None, link.ParticleEmitterByIndex(1))
        # self.assertFalse(link.particle_emitter_name_exists(""))
        # self.assertFalse(link.particle_emitter_name_exists("default"))
        # self.assertEqual(None, link.ParticleEmitterByName("no_such_emitter"))

        self.assertEqual(0, link.projector_count())
        self.assertEqual(None, link.projector_by_index(0))
        self.assertEqual(None, link.projector_by_index(1))
        self.assertFalse(link.projector_name_exists(""))
        self.assertFalse(link.projector_name_exists("default"))
        self.assertEqual(None, link.projector_by_name("no_such_projector"))

        self.assertFalse(link.enable_wind())
        link.set_enable_wind(True)
        self.assertTrue(link.enable_wind())

        self.assertEqual(0, link.sensor_count())
        self.assertEqual(None, link.sensor_by_index(0))
        self.assertEqual(None, link.sensor_by_index(1))
        self.assertEqual(None, link.sensor_by_name("empty"))
        self.assertFalse(link.sensor_name_exists(""))
        self.assertFalse(link.sensor_name_exists("default"))

        self.assertEqual(Pose3d.ZERO, link.raw_pose())
        self.assertFalse(link.pose_relative_to())

        semantic_pose = link.semantic_pose()
        self.assertEqual(Pose3d.ZERO, semantic_pose.raw_pose())
        self.assertFalse(semantic_pose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semantic_pose.resolve()

        link.set_raw_pose(Pose3d(10, 20, 30, 0, math.pi, 0))
        self.assertEqual(Pose3d(10, 20, 30, 0, math.pi, 0), link.raw_pose())

        link.set_pose_relative_to("model")
        self.assertEqual("model", link.pose_relative_to())

        semantic_pose = link.semantic_pose()
        self.assertEqual(link.raw_pose(), semantic_pose.raw_pose())
        self.assertEqual("model", semantic_pose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semantic_pose.resolve()

        # Get the default inertial
        inertial = link.inertial()
        self.assertAlmostEqual(1.0, inertial.mass_matrix().mass())
        self.assertAlmostEqual(1.0, inertial.mass_matrix().diagonal_moments().x())
        self.assertAlmostEqual(1.0, inertial.mass_matrix().diagonal_moments().y())
        self.assertAlmostEqual(1.0, inertial.mass_matrix().diagonal_moments().z())
        self.assertAlmostEqual(0.0, inertial.mass_matrix().off_diagonal_moments().x())
        self.assertAlmostEqual(0.0, inertial.mass_matrix().off_diagonal_moments().y())
        self.assertAlmostEqual(0.0, inertial.mass_matrix().off_diagonal_moments().z())
        self.assertTrue(inertial.mass_matrix().is_valid())

        self.assertEqual(0, link.collision_count())
        self.assertEqual(None, link.collision_by_index(0))
        self.assertEqual(None, link.collision_by_index(1))
        self.assertFalse(link.collision_name_exists(""))
        self.assertFalse(link.collision_name_exists("default"))

        inertial2 = Inertiald(
            MassMatrix3d(2.3,
                         Vector3d(1.4, 2.3, 3.2),
                         Vector3d(0.1, 0.2, 0.3)),
            Pose3d(1, 2, 3, 0, 0, 0))

        self.assertTrue(link.set_inertial(inertial2))

        inertial = link.inertial()
        self.assertAlmostEqual(2.3, inertial2.mass_matrix().mass())
        self.assertAlmostEqual(2.3, inertial.mass_matrix().mass())
        self.assertAlmostEqual(1.4, inertial.mass_matrix().diagonal_moments().x())
        self.assertAlmostEqual(2.3, inertial.mass_matrix().diagonal_moments().y())
        self.assertAlmostEqual(3.2, inertial.mass_matrix().diagonal_moments().z())
        self.assertAlmostEqual(0.1, inertial.mass_matrix().off_diagonal_moments().x())
        self.assertAlmostEqual(0.2, inertial.mass_matrix().off_diagonal_moments().y())
        self.assertAlmostEqual(0.3, inertial.mass_matrix().off_diagonal_moments().z())
        self.assertTrue(inertial.mass_matrix().is_valid())


    def test_copy_construction(self):
        link = Link()
        link.set_name("test_link")

        link2 = Link(link)
        self.assertEqual("test_link", link2.name())


    def test_deepcopy(self):
        link = Link()
        link.set_name("test_link")

        link2 = copy.deepcopy(link)
        self.assertEqual("test_link", link2.name())


    def test_invalid_inertial(self):
        link = Link()
        self.assertFalse(link.name())

        invalidInertial = Inertiald(
            MassMatrix3d(2.3, Vector3d(0.1, 0.2, 0.3),
                         Vector3d(1.2, 2.3, 3.4)),
            Pose3d(1, 2, 3, 0, 0, 0))

        self.assertFalse(link.set_inertial(invalidInertial))

        inertial = link.inertial()
        self.assertAlmostEqual(2.3, inertial.mass_matrix().mass())
        self.assertAlmostEqual(0.1, inertial.mass_matrix().diagonal_moments().x())
        self.assertAlmostEqual(0.2, inertial.mass_matrix().diagonal_moments().y())
        self.assertAlmostEqual(0.3, inertial.mass_matrix().diagonal_moments().z())
        self.assertAlmostEqual(1.2, inertial.mass_matrix().off_diagonal_moments().x())
        self.assertAlmostEqual(2.3, inertial.mass_matrix().off_diagonal_moments().y())
        self.assertAlmostEqual(3.4, inertial.mass_matrix().off_diagonal_moments().z())
        self.assertFalse(link.inertial().mass_matrix().is_valid())


    def test_add_collision(self):
        link = Link()
        self.assertEqual(0, link.collision_count())

        collision = Collision()
        collision.set_name("collision1")
        self.assertTrue(link.add_collision(collision))
        self.assertEqual(1, link.collision_count())
        self.assertFalse(link.add_collision(collision))
        self.assertEqual(1, link.collision_count())

        link.clear_collisions()
        self.assertEqual(0, link.collision_count())

        self.assertTrue(link.add_collision(collision))
        self.assertEqual(1, link.collision_count())
        collisionFromLink = link.collision_by_index(0)
        self.assertNotEqual(None, collisionFromLink)
        self.assertEqual(collisionFromLink.name(), collision.name())

    def test_add_visual(self):
        link = Link()
        self.assertEqual(0, link.visual_count())

        visual = Visual()
        visual.set_name("visual1")
        self.assertTrue(link.add_visual(visual))
        self.assertEqual(1, link.visual_count())
        self.assertFalse(link.add_visual(visual))
        self.assertEqual(1, link.visual_count())

        link.clear_visuals()
        self.assertEqual(0, link.visual_count())

        self.assertTrue(link.add_visual(visual))
        self.assertEqual(1, link.visual_count())
        visualFromLink = link.visual_by_index(0)
        self.assertNotEqual(None, visualFromLink)
        self.assertEqual(visualFromLink.name(), visual.name())

    def test_add_light(self):
        link = Link()
        self.assertEqual(0, link.light_count())

        light = Light()
        light.set_name("light1")
        self.assertTrue(link.add_light(light))
        self.assertEqual(1, link.light_count())
        self.assertFalse(link.add_light(light))
        self.assertEqual(1, link.light_count())

        link.clear_lights()
        self.assertEqual(0, link.light_count())

        self.assertTrue(link.add_light(light))
        self.assertEqual(1, link.light_count())
        lightFromLink = link.light_by_index(0)
        self.assertNotEqual(None, lightFromLink)
        self.assertEqual(lightFromLink.name(), light.name())

    def test_add_sensor(self):
        link = Link()
        self.assertEqual(0, link.sensor_count())

        sensor = Sensor()
        sensor.set_name("sensor1")
        self.assertTrue(link.add_sensor(sensor))
        self.assertEqual(1, link.sensor_count())
        self.assertFalse(link.add_sensor(sensor))
        self.assertEqual(1, link.sensor_count())

        link.clear_sensors()
        self.assertEqual(0, link.sensor_count())

        self.assertTrue(link.add_sensor(sensor))
        self.assertEqual(1, link.sensor_count())
        sensorFromLink = link.sensor_by_index(0)
        self.assertNotEqual(None, sensorFromLink)
        self.assertEqual(sensorFromLink.name(), sensor.name())

    def test_mutable_by_index(self):
        link = Link()
        link.set_name("my-name")

        visual = Visual()
        visual.set_name("visual1")
        self.assertTrue(link.add_visual(visual))

        collision = Collision()
        collision.set_name("collision1")
        self.assertTrue(link.add_collision(collision))

        light = Light()
        light.set_name("light1")
        self.assertTrue(link.add_light(light))

        sensor = Sensor()
        sensor.set_name("sensor1")
        self.assertTrue(link.add_sensor(sensor))

        # sdf::ParticleEmitter pe
        # pe.set_name("pe1")
        # self.assertTrue(link.AddParticleEmitter(pe))

        projector = Projector()
        projector.set_name("projector1")
        self.assertTrue(link.add_projector(projector))

        # Modify the visual
        v = link.visual_by_index(0)
        self.assertNotEqual(None, v)
        self.assertEqual("visual1", v.name())
        v.set_name("visual2")
        self.assertEqual("visual2", link.visual_by_index(0).name())

        # Modify the collision
        c = link.collision_by_index(0)
        self.assertNotEqual(None, c)
        self.assertEqual("collision1", c.name())
        c.set_name("collision2")
        self.assertEqual("collision2", link.collision_by_index(0).name())

        # Modify the light
        l = link.light_by_index(0)
        self.assertNotEqual(None, l)
        self.assertEqual("light1", l.name())
        l.set_name("light2")
        self.assertEqual("light2", link.light_by_index(0).name())

        # Modify the sensor
        s = link.sensor_by_index(0)
        self.assertNotEqual(None, s)
        self.assertEqual("sensor1", s.name())
        s.set_name("sensor2")
        self.assertEqual("sensor2", link.sensor_by_index(0).name())

        # # Modify the particle emitter
        # sdf::ParticleEmitter *p = link.ParticleEmitterByIndex(0)
        # self.assertNotEqual(None, p)
        # self.assertEqual("pe1", p.name())
        # p.set_name("pe2")
        # self.assertEqual("pe2", link.ParticleEmitterByIndex(0).name())

        # Modify the projector
        pr = link.projector_by_index(0)
        self.assertNotEqual(None, pr)
        self.assertEqual("projector1", pr.name())
        pr.set_name("projector2");
        self.assertEqual("projector2", link.projector_by_index(0).name());

    def test_mutable_by_name(self):
        link = Link()
        link.set_name("my-name")

        visual = Visual()
        visual.set_name("visual1")
        self.assertTrue(link.add_visual(visual))

        collision = Collision()
        collision.set_name("collision1")
        self.assertTrue(link.add_collision(collision))

        light = Light()
        light.set_name("light1")
        self.assertTrue(link.add_light(light))

        sensor = Sensor()
        sensor.set_name("sensor1")
        self.assertTrue(link.add_sensor(sensor))

        # sdf::ParticleEmitter pe
        # pe.set_name("pe1")
        # self.assertTrue(link.AddParticleEmitter(pe))

        projector = Projector()
        projector.set_name("projector1")
        self.assertTrue(link.add_projector(projector))

        # Modify the visual
        v = link.visual_by_name("visual1")
        self.assertNotEqual(None, v)
        self.assertEqual("visual1", v.name())
        v.set_name("visual2")
        self.assertFalse(link.visual_name_exists("visual1"))
        self.assertTrue(link.visual_name_exists("visual2"))

        # Modify the collision
        c = link.collision_by_name("collision1")
        self.assertNotEqual(None, c)
        self.assertEqual("collision1", c.name())
        c.set_name("collision2")
        self.assertFalse(link.collision_name_exists("collision1"))
        self.assertTrue(link.collision_name_exists("collision2"))

        # Modify the light
        l = link.light_by_name("light1")
        self.assertNotEqual(None, l)
        self.assertEqual("light1", l.name())
        l.set_name("light2")
        self.assertFalse(link.light_name_exists("light1"))
        self.assertTrue(link.light_name_exists("light2"))

        # Modify the sensor
        s = link.sensor_by_name("sensor1")
        self.assertNotEqual(None, s)
        self.assertEqual("sensor1", s.name())
        s.set_name("sensor2")
        self.assertFalse(link.sensor_name_exists("sensor1"))
        self.assertTrue(link.sensor_name_exists("sensor2"))

        # // Modify the particle emitter
        # sdf::ParticleEmitter *p = link.ParticleEmitterByName("pe1")
        # self.assertNotEqual(None, p)
        # self.assertEqual("pe1", p.name())
        # p.set_name("pe2")
        # self.assertFalse(link.particle_emitter_name_exists("pe1"))
        # self.assertTrue(link.particle_emitter_name_exists("pe2"))

        # Modify the projector
        pr = link.projector_by_name("projector1");
        self.assertNotEqual(None, pr);
        self.assertEqual("projector1", pr.name());
        pr.set_name("projector2");
        self.assertFalse(link.projector_name_exists("projector1"));
        self.assertTrue(link.projector_name_exists("projector2"));

if __name__ == '__main__':
    unittest.main()
