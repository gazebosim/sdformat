/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include "sdf/Surface.hh"

/////////////////////////////////////////////////
TEST(DOMsurface, DefaultConstruction)
{
  sdf::Surface surface;
  EXPECT_EQ(nullptr, surface.Element());
  EXPECT_EQ(surface.Contact()->CollideBitmask(), 0xFF);
  EXPECT_EQ(surface.Contact()->Element(), nullptr);
}

/////////////////////////////////////////////////
TEST(DOMsurface, CopyOperator)
{
  sdf::Surface surface1;
  sdf::Contact contact;
  contact.SetCollideBitmask(0x12);
  surface1.SetContact(contact);

  sdf::Surface surface2(surface1);
  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
}

/////////////////////////////////////////////////
TEST(DOMsurface, CopyAssignmentOperator)
{
  sdf::Surface surface1;
  sdf::Contact contact;
  contact.SetCollideBitmask(0x12);
  surface1.SetContact(contact);

  sdf::Surface surface2 = surface1;
  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
}

/////////////////////////////////////////////////
TEST(DOMsurface, CopyAssignmentAfterMove)
{
  sdf::Surface surface1;
  sdf::Surface surface2;

  sdf::Contact contact1;
  contact1.SetCollideBitmask(0x12);
  surface1.SetContact(contact1);
  sdf::Contact contact2;
  contact2.SetCollideBitmask(0x34);
  surface2.SetContact(contact2);

  sdf::Surface tmp = std::move(surface1);
  surface1 = surface2;
  surface2 = tmp;

  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
  EXPECT_EQ(surface1.Contact()->CollideBitmask(), 0x34);
}

/////////////////////////////////////////////////
TEST(DOMcontact, DefaultConstruction)
{
  sdf::Contact contact;
  EXPECT_EQ(nullptr, contact.Element());
  EXPECT_EQ(contact.CollideBitmask(), 0xFF);
  EXPECT_EQ(contact.Element(), nullptr);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CopyOperator)
{
  sdf::Contact contact1;
  contact1.SetCollideBitmask(0x12);

  sdf::Contact contact2(contact1);
  EXPECT_EQ(contact2.CollideBitmask(), 0x12);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CopyAssignmentOperator)
{
  sdf::Contact contact1;
  contact1.SetCollideBitmask(0x12);

  sdf::Contact contact2 = contact1;
  EXPECT_EQ(contact2.CollideBitmask(), 0x12);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CopyAssignmentAfterMove)
{
  sdf::Contact contact1;
  sdf::Contact contact2;

  contact1.SetCollideBitmask(0x12);
  contact2.SetCollideBitmask(0x34);

  sdf::Contact tmp = std::move(contact1);
  contact1 = contact2;
  contact2 = tmp;

  EXPECT_EQ(contact2.CollideBitmask(), 0x12);
  EXPECT_EQ(contact1.CollideBitmask(), 0x34);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CollideBitmask)
{
  sdf::Contact contact;
  contact.SetCollideBitmask(0x67);
  EXPECT_EQ(contact.CollideBitmask(), 0x67);
}

