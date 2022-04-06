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
from sdformat import Surface, Contact
import unittest


class SurfaceTEST(unittest.TestCase):

  def test_default_construction(self):
      surface = Surface()
      self.assertEqual(surface.contact().collide_bitmask(), 0xFF)


  def test_copy_construction(self):
    surface1 = Surface()
    contact = Contact()
    contact.set_collide_bitmask(0x12)
    surface1.set_contact(contact)

    surface2 = Surface(surface1)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)


  def test_copy_assignment(self):
    surface1 = Surface()
    contact = Contact()
    contact.set_collide_bitmask(0x12)
    surface1.set_contact(contact)

    surface2 = copy.deepcopy(surface1)
    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)

  def test_copy_assignment_after_move(self):
    surface1 = Surface()
    surface2 = Surface()

    contact1 = Contact()
    contact1.set_collide_bitmask(0x12)
    surface1.set_contact(contact1)
    contact2 = Contact()
    contact2.set_collide_bitmask(0x34)
    surface2.set_contact(contact2)

    tmp = copy.deepcopy(surface1)
    surface1 = surface2
    surface2 = tmp

    self.assertEqual(surface2.contact().collide_bitmask(), 0x12)
    self.assertEqual(surface1.contact().collide_bitmask(), 0x34)


  def test_default_contact_construction(self):
    contact = Contact()
    self.assertEqual(contact.collide_bitmask(), 0xFF)


  def test_copy_contact_construction(self):
    contact1 = Contact()
    contact1.set_collide_bitmask(0x12)

    contact2 = Contact(contact1)
    self.assertEqual(contact2.collide_bitmask(), 0x12)


  def test_copy_assignment(self):
    contact1 = Contact()
    contact1.set_collide_bitmask(0x12)

    contact2 = Contact(contact1)
    self.assertEqual(contact2.collide_bitmask(), 0x12)


  def test_copy_contact_assignment_after_move(self):
    contact1 = Contact()
    contact2 = Contact()

    contact1.set_collide_bitmask(0x12)
    contact2.set_collide_bitmask(0x34)

    tmp = copy.deepcopy(contact1)
    contact1 = contact2
    contact2 = tmp

    self.assertEqual(contact2.collide_bitmask(), 0x12)
    self.assertEqual(contact1.collide_bitmask(), 0x34)


  def test_collide_bitmask(self):
    contact = Contact()
    contact.set_collide_bitmask(0x67)
    self.assertEqual(contact.collide_bitmask(), 0x67)


if __name__ == '__main__':
    unittest.main()
