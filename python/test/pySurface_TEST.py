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
from gz_test_deps.math import Vector3d
from gz_test_deps.sdformat import Surface, Contact, Friction, ODE, \
                                  BulletFriction, Torsional
import unittest


class SurfaceTEST(unittest.TestCase):

  def test_default_construction(self):
      surface = Surface()
      self.assertEqual(surface.contact().collide_bitmask(), 0xFF)


  def test_assigment_construction(self):
    surface1 = Surface()
    contact = Contact()
    ode = ODE()
    ode.set_mu(0.1)
    ode.set_mu2(0.2)
    ode.set_slip1(3)
    ode.set_slip2(4)
    ode.set_fdir1(Vector3d(1, 2, 3))

    bullet = BulletFriction()
    bullet.set_friction(0.11)
    bullet.set_friction2(0.22)
    bullet.set_fdir1(Vector3d(3, 2, 1))
    bullet.set_rolling_friction(0.33)

    torsional = Torsional()
    torsional.set_coefficient(1.2)
    torsional.set_use_patch_radius(True)
    torsional.set_patch_radius(0.5)
    torsional.set_surface_radius(0.15)
    torsional.set_ode_slip(0.01)

    friction = Friction()
    friction.set_ode(ode)
    friction.set_bullet_friction(bullet)
    friction.set_torsional(torsional)
    contact.set_collide_bitmask(0x12)
    surface1.set_contact(contact)
    surface1.set_friction(friction)

    surface2 = surface1
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)
    self.assertEqual(surface2.friction().ode().mu(), 0.1)
    self.assertEqual(surface2.friction().ode().mu2(), 0.2)
    self.assertEqual(surface2.friction().ode().slip1(), 3)
    self.assertEqual(surface2.friction().ode().slip2(), 4)
    self.assertEqual(surface2.friction().ode().fdir1(),
                     Vector3d(1, 2, 3))
    self.assertEqual(surface2.friction().bullet_friction().friction(), 0.11)
    self.assertEqual(surface2.friction().bullet_friction().friction2(), 0.22)
    self.assertEqual(surface2.friction().bullet_friction().fdir1(),
                     Vector3d(3, 2, 1))
    self.assertEqual(surface2.friction().bullet_friction().rolling_friction(),
                     0.33)
    self.assertEqual(surface2.friction().torsional().coefficient(), 1.2)
    self.assertTrue(surface2.friction().torsional().use_patch_radius())
    self.assertEqual(surface2.friction().torsional().patch_radius(), 0.5)
    self.assertEqual(surface2.friction().torsional().surface_radius(), 0.15)
    self.assertEqual(surface2.friction().torsional().ode_slip(), 0.01)

    contact.set_collide_bitmask(0x21)
    surface1.set_contact(contact)
    self.assertEqual(surface1.contact().collide_bitmask(), 0x21)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x21)

    ode.set_mu(1.1)
    ode.set_mu2(1.2)
    ode.set_slip1(3.1)
    ode.set_slip2(4.1)
    ode.set_fdir1(Vector3d(1.1, 2.1, 3.1))
    friction.set_ode(ode)

    bullet.set_friction(0.33)
    bullet.set_friction2(0.45)
    bullet.set_fdir1(Vector3d(0, 1, 2))
    bullet.set_rolling_friction(0.03)
    friction.set_bullet_friction(bullet)

    torsional.set_coefficient(2.1)
    torsional.set_use_patch_radius(False)
    torsional.set_patch_radius(0.1)
    torsional.set_surface_radius(0.9)
    torsional.set_ode_slip(0.7)
    friction.set_torsional(torsional)

    surface1.set_friction(friction)
    self.assertEqual(surface1.friction().ode().mu(), 1.1)
    self.assertEqual(surface1.friction().ode().mu2(), 1.2)
    self.assertEqual(surface1.friction().ode().slip1(), 3.1)
    self.assertEqual(surface1.friction().ode().slip2(), 4.1)
    self.assertEqual(surface1.friction().ode().fdir1(),
                     Vector3d(1.1, 2.1, 3.1))
    self.assertEqual(surface2.friction().ode().mu(), 1.1)
    self.assertEqual(surface2.friction().ode().mu2(), 1.2)
    self.assertEqual(surface2.friction().ode().slip1(), 3.1)
    self.assertEqual(surface2.friction().ode().slip2(), 4.1)
    self.assertEqual(surface2.friction().ode().fdir1(),
                     Vector3d(1.1, 2.1, 3.1))

    self.assertEqual(surface1.friction().bullet_friction().friction(), 0.33)
    self.assertEqual(surface1.friction().bullet_friction().friction2(), 0.45)
    self.assertEqual(surface1.friction().bullet_friction().fdir1(),
                     Vector3d(0, 1, 2))
    self.assertEqual(surface1.friction().bullet_friction().rolling_friction(),
                     0.03)
    self.assertEqual(surface2.friction().bullet_friction().friction(), 0.33)
    self.assertEqual(surface2.friction().bullet_friction().friction2(), 0.45)
    self.assertEqual(surface2.friction().bullet_friction().fdir1(),
                     Vector3d(0, 1, 2))
    self.assertEqual(surface2.friction().bullet_friction().rolling_friction(),
                     0.03)

    self.assertEqual(surface1.friction().torsional().coefficient(), 2.1)
    self.assertFalse(surface1.friction().torsional().use_patch_radius())
    self.assertEqual(surface1.friction().torsional().patch_radius(), 0.1)
    self.assertEqual(surface1.friction().torsional().surface_radius(), 0.9)
    self.assertEqual(surface1.friction().torsional().ode_slip(), 0.7)
    self.assertEqual(surface2.friction().torsional().coefficient(), 2.1)
    self.assertFalse(surface2.friction().torsional().use_patch_radius())
    self.assertEqual(surface2.friction().torsional().patch_radius(), 0.1)
    self.assertEqual(surface2.friction().torsional().surface_radius(), 0.9)
    self.assertEqual(surface2.friction().torsional().ode_slip(), 0.7)

  def test_copy_construction(self):
    surface1 = Surface()
    contact = Contact()
    ode = ODE()
    ode.set_mu(0.1)
    ode.set_mu2(0.2)
    ode.set_slip1(3)
    ode.set_slip2(4)
    ode.set_fdir1(Vector3d(1, 2, 3))
    bullet = BulletFriction()
    bullet.set_friction(0.11)
    bullet.set_friction2(0.22)
    bullet.set_fdir1(Vector3d(3, 2, 1))
    bullet.set_rolling_friction(0.33)
    torsional = Torsional()
    torsional.set_coefficient(1.2)
    torsional.set_use_patch_radius(True)
    torsional.set_patch_radius(0.5)
    torsional.set_surface_radius(0.15)
    torsional.set_ode_slip(0.01)
    friction = Friction()
    friction.set_ode(ode)
    friction.set_bullet_friction(bullet)
    friction.set_torsional(torsional)
    contact.set_collide_bitmask(0x12)
    surface1.set_contact(contact)
    surface1.set_friction(friction)

    surface2 = Surface(surface1)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)
    self.assertEqual(surface2.friction().ode().mu(), 0.1)
    self.assertEqual(surface2.friction().ode().mu2(), 0.2)
    self.assertEqual(surface2.friction().ode().slip1(), 3)
    self.assertEqual(surface2.friction().ode().slip2(), 4)
    self.assertEqual(surface2.friction().ode().fdir1(),
                     Vector3d(1, 2, 3))
    self.assertEqual(surface2.friction().bullet_friction().friction(), 0.11)
    self.assertEqual(surface2.friction().bullet_friction().friction2(), 0.22)
    self.assertEqual(surface2.friction().bullet_friction().fdir1(),
                     Vector3d(3, 2, 1))
    self.assertEqual(surface2.friction().bullet_friction().rolling_friction(),
                     0.33)
    self.assertEqual(surface2.friction().torsional().coefficient(), 1.2)
    self.assertTrue(surface2.friction().torsional().use_patch_radius())
    self.assertEqual(surface2.friction().torsional().patch_radius(), 0.5)
    self.assertEqual(surface2.friction().torsional().surface_radius(), 0.15)
    self.assertEqual(surface2.friction().torsional().ode_slip(), 0.01)

    contact.set_collide_bitmask(0x21)
    surface1.set_contact(contact)
    self.assertEqual(surface1.contact().collide_bitmask(), 0x21)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)

    ode.set_mu(1.1)
    ode.set_mu2(1.2)
    ode.set_slip1(3.1)
    ode.set_slip2(4.1)
    ode.set_fdir1(Vector3d(1.1, 2.1, 3.1))
    friction.set_ode(ode)

    bullet.set_friction(0.33)
    bullet.set_friction2(0.45)
    bullet.set_fdir1(Vector3d(0, 1, 2))
    bullet.set_rolling_friction(0.03)
    friction.set_bullet_friction(bullet)

    torsional.set_coefficient(2.1)
    torsional.set_use_patch_radius(False)
    torsional.set_patch_radius(0.1)
    torsional.set_surface_radius(0.9)
    torsional.set_ode_slip(0.7)
    friction.set_torsional(torsional)

    surface1.set_friction(friction)
    self.assertEqual(surface1.friction().ode().mu(), 1.1)
    self.assertEqual(surface1.friction().ode().mu2(), 1.2)
    self.assertEqual(surface1.friction().ode().slip1(), 3.1)
    self.assertEqual(surface1.friction().ode().slip2(), 4.1)
    self.assertEqual(surface1.friction().ode().fdir1(),
                     Vector3d(1.1, 2.1, 3.1))
    self.assertEqual(surface2.friction().ode().mu(), 0.1)
    self.assertEqual(surface2.friction().ode().mu2(), 0.2)
    self.assertEqual(surface2.friction().ode().slip1(), 3)
    self.assertEqual(surface2.friction().ode().slip2(), 4)
    self.assertEqual(surface2.friction().ode().fdir1(),
                     Vector3d(1, 2, 3))

    self.assertEqual(surface1.friction().bullet_friction().friction(), 0.33)
    self.assertEqual(surface1.friction().bullet_friction().friction2(), 0.45)
    self.assertEqual(surface1.friction().bullet_friction().fdir1(),
                     Vector3d(0, 1, 2))
    self.assertEqual(surface1.friction().bullet_friction().rolling_friction(),
                     0.03)
    self.assertEqual(surface2.friction().bullet_friction().friction(), 0.11)
    self.assertEqual(surface2.friction().bullet_friction().friction2(), 0.22)
    self.assertEqual(surface2.friction().bullet_friction().fdir1(),
                     Vector3d(3, 2, 1))
    self.assertEqual(surface2.friction().bullet_friction().rolling_friction(),
                     0.33)

    self.assertEqual(surface1.friction().torsional().coefficient(), 2.1)
    self.assertFalse(surface1.friction().torsional().use_patch_radius())
    self.assertEqual(surface1.friction().torsional().patch_radius(), 0.1)
    self.assertEqual(surface1.friction().torsional().surface_radius(), 0.9)
    self.assertEqual(surface1.friction().torsional().ode_slip(), 0.7)
    self.assertEqual(surface2.friction().torsional().coefficient(), 1.2)
    self.assertTrue(surface2.friction().torsional().use_patch_radius())
    self.assertEqual(surface2.friction().torsional().patch_radius(), 0.5)
    self.assertEqual(surface2.friction().torsional().surface_radius(), 0.15)
    self.assertEqual(surface2.friction().torsional().ode_slip(), 0.01)

  def test_deepcopy(self):
    surface1 = Surface()
    contact = Contact()
    ode = ODE()
    ode.set_mu(0.1)
    ode.set_mu2(0.2)
    ode.set_slip1(3)
    ode.set_slip2(4)
    ode.set_fdir1(Vector3d(1, 2, 3))
    bullet = BulletFriction()
    bullet.set_friction(0.11)
    bullet.set_friction2(0.22)
    bullet.set_fdir1(Vector3d(3, 2, 1))
    bullet.set_rolling_friction(0.33)
    torsional = Torsional()
    torsional.set_coefficient(1.2)
    torsional.set_use_patch_radius(True)
    torsional.set_patch_radius(0.5)
    torsional.set_surface_radius(0.15)
    torsional.set_ode_slip(0.01)
    friction = Friction()
    friction.set_ode(ode)
    friction.set_bullet_friction(bullet)
    friction.set_torsional(torsional)
    contact.set_collide_bitmask(0x12)
    surface1.set_contact(contact)
    surface1.set_friction(friction)

    surface2 = copy.deepcopy(surface1)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)
    self.assertEqual(surface2.friction().ode().mu(), 0.1)
    self.assertEqual(surface2.friction().ode().mu2(), 0.2)
    self.assertEqual(surface2.friction().ode().slip1(), 3)
    self.assertEqual(surface2.friction().ode().slip2(), 4)
    self.assertEqual(surface2.friction().ode().fdir1(),
                     Vector3d(1, 2, 3))
    self.assertEqual(surface2.friction().bullet_friction().friction(), 0.11)
    self.assertEqual(surface2.friction().bullet_friction().friction2(), 0.22)
    self.assertEqual(surface2.friction().bullet_friction().fdir1(),
                     Vector3d(3, 2, 1))
    self.assertEqual(surface2.friction().bullet_friction().rolling_friction(),
                     0.33)
    self.assertEqual(surface2.friction().torsional().coefficient(), 1.2)
    self.assertTrue(surface2.friction().torsional().use_patch_radius())
    self.assertEqual(surface2.friction().torsional().patch_radius(), 0.5)
    self.assertEqual(surface2.friction().torsional().surface_radius(), 0.15)
    self.assertEqual(surface2.friction().torsional().ode_slip(), 0.01)

    contact.set_collide_bitmask(0x21)
    surface1.set_contact(contact)
    self.assertEqual(surface1.contact().collide_bitmask(), 0x21)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)

    ode.set_mu(1.1)
    ode.set_mu2(1.2)
    ode.set_slip1(3.1)
    ode.set_slip2(4.1)
    ode.set_fdir1(Vector3d(1.1, 2.1, 3.1))
    friction.set_ode(ode)
    bullet.set_friction(0.33)
    bullet.set_friction2(0.45)
    bullet.set_fdir1(Vector3d(0, 1, 2))
    bullet.set_rolling_friction(0.03)
    friction.set_bullet_friction(bullet)
    torsional.set_coefficient(2.1)
    torsional.set_use_patch_radius(False)
    torsional.set_patch_radius(0.1)
    torsional.set_surface_radius(0.9)
    torsional.set_ode_slip(0.7)
    friction.set_torsional(torsional)
    surface1.set_friction(friction)

    self.assertEqual(surface1.friction().ode().mu(), 1.1)
    self.assertEqual(surface1.friction().ode().mu2(), 1.2)
    self.assertEqual(surface1.friction().ode().slip1(), 3.1)
    self.assertEqual(surface1.friction().ode().slip2(), 4.1)
    self.assertEqual(surface1.friction().ode().fdir1(),
                     Vector3d(1.1, 2.1, 3.1))
    self.assertEqual(surface2.friction().ode().mu(), 0.1)
    self.assertEqual(surface2.friction().ode().mu2(), 0.2)
    self.assertEqual(surface2.friction().ode().slip1(), 3)
    self.assertEqual(surface2.friction().ode().slip2(), 4)
    self.assertEqual(surface2.friction().ode().fdir1(),
                     Vector3d(1, 2, 3))

    self.assertEqual(surface1.friction().bullet_friction().friction(), 0.33)
    self.assertEqual(surface1.friction().bullet_friction().friction2(), 0.45)
    self.assertEqual(surface1.friction().bullet_friction().fdir1(),
                     Vector3d(0, 1, 2))
    self.assertEqual(surface1.friction().bullet_friction().rolling_friction(),
                     0.03)
    self.assertEqual(surface2.friction().bullet_friction().friction(), 0.11)
    self.assertEqual(surface2.friction().bullet_friction().friction2(), 0.22)
    self.assertEqual(surface2.friction().bullet_friction().fdir1(),
                     Vector3d(3, 2, 1))
    self.assertEqual(surface2.friction().bullet_friction().rolling_friction(),
                     0.33)

    self.assertEqual(surface1.friction().torsional().coefficient(), 2.1)
    self.assertFalse(surface1.friction().torsional().use_patch_radius())
    self.assertEqual(surface1.friction().torsional().patch_radius(), 0.1)
    self.assertEqual(surface1.friction().torsional().surface_radius(), 0.9)
    self.assertEqual(surface1.friction().torsional().ode_slip(), 0.7)
    self.assertEqual(surface2.friction().torsional().coefficient(), 1.2)
    self.assertTrue(surface2.friction().torsional().use_patch_radius())
    self.assertEqual(surface2.friction().torsional().patch_radius(), 0.5)
    self.assertEqual(surface2.friction().torsional().surface_radius(), 0.15)
    self.assertEqual(surface2.friction().torsional().ode_slip(), 0.01)

  def test_default_contact_construction(self):
    contact = Contact()
    self.assertEqual(contact.collide_bitmask(), 0xFF)


  def test_copy_contact_construction(self):
    contact1 = Contact()
    contact1.set_collide_bitmask(0x12)

    contact2 = Contact(contact1)
    self.assertEqual(contact2.collide_bitmask(), 0x12)

  def test_default_ode_construction(self):
    ode = ODE()
    self.assertEqual(ode.mu(), 1.0)
    self.assertEqual(ode.mu2(), 1.0)
    self.assertEqual(ode.slip1(), 0)
    self.assertEqual(ode.slip2(), 0)
    self.assertEqual(ode.fdir1(),
                     Vector3d(0, 0, 0))

  def test_default_bullet_friction_construction(self):
    bullet = BulletFriction()
    self.assertEqual(bullet.friction(), 1.0)
    self.assertEqual(bullet.friction2(), 1.0)
    self.assertEqual(bullet.fdir1(),
                     Vector3d(0, 0, 0))
    self.assertEqual(bullet.rolling_friction(), 1.0)

  def test_default_torsional_construction(self):
    torsional = Torsional()
    self.assertEqual(torsional.coefficient(), 1.0)
    self.assertTrue(torsional.use_patch_radius())
    self.assertEqual(torsional.patch_radius(), 0.0)
    self.assertEqual(torsional.surface_radius(), 0.0)
    self.assertEqual(torsional.ode_slip(), 0.0)

if __name__ == '__main__':
    unittest.main()
