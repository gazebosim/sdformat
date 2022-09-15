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
from gz.math7 import Vector3d
from sdformat import Surface, Contact, Friction, ODE
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
    friction = Friction()
    friction.set_ode(ode)
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


  def test_copy_construction(self):
    surface1 = Surface()
    contact = Contact()
    ode = ODE()
    ode.set_mu(0.1)
    ode.set_mu2(0.2)
    ode.set_slip1(3)
    ode.set_slip2(4)
    ode.set_fdir1(Vector3d(1, 2, 3))
    friction = Friction()
    friction.set_ode(ode)
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


  def test_deepcopy(self):
    surface1 = Surface()
    contact = Contact()
    ode = ODE()
    ode.set_mu(0.1)
    ode.set_mu2(0.2)
    ode.set_slip1(3)
    ode.set_slip2(4)
    ode.set_fdir1(Vector3d(1, 2, 3))
    friction = Friction()
    friction.set_ode(ode)
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


if __name__ == '__main__':
    unittest.main()
