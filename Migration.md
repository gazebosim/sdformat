# Migration Guide for SDF Protocol
This document contains information about migrating
between different versions of the SDF protocol.
The SDF protocol version number is specified in the `version` attribute
of the `sdf` element (1.4, 1.5, 1.6, etc.)
and is distinct from sdformat library version
(2.3, 3.0, 4.0, etc.).

# Note on backward compatibility
There are `*.convert` files that allow old sdf files to be migrated
forward programmatically.
This document aims to contain similar information to those files
but with improved human-readability..

## SDFormat 3.x to 4.x

### Additions

1. **New SDF protocol version 1.6**
    + Details about the 1.5 to 1.6 transition are explained below in this same
      document

### Modifications

1. **Boost pointers and boost::function**
    + All boost pointers, boost::function in the public API have been replaced
      by their std:: equivalents (C++11 standard)

1. **`gravity` and `magnetic_field` elements are moved  from `physics` to `world`**
    + In physics element: gravity and magnetic_field tags have been moved
      from Physics to World element.
    + [BitBucket pull request 247](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/247)
    + [gazebo pull request 2090](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2090)

1. **New noise for IMU**
    + A new style for representing the noise properties of an `imu` was implemented
      in [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)
      for sdf 1.5 and the old style was declared as deprecated.
      The old style has been removed from sdf 1.6 with the conversion script
      updating to the new style.
    + [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)
    + [BitBucket pull request 243](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/243)
    + [BitBucket pull request 244](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/244)

1. **Lump:: prefix in link names**
    + Changed to \_fixed_joint_lump__ to avoid confusion with scoped names
    + [BitBucket pull request 245](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/245)

## SDF protocol 1.5 to 1.6

### Additions

1. **link.sdf** `enable_wind` element
    + description: If true, the link is affected by the wind
    + type: bool
    + default: false
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **link.sdf** `light` element
    + included from `light.sdf` with required="*",
      so a link can have any number of attached lights.
    + [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

1. **model.sdf** `enable_wind` element
    + description: If set to true, all links in the model will be affected by
      the wind.  Can be overriden by the link wind property.
    + type: bool
    + default: false
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **model_state.sdf** `scale` element
    + description: Scale for the 3 dimensions of the model.
    + type: vector3
    + default: "1 1 1"
    + required: 0
    + [BitBucket pull request 246](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/246)

1. **physics.sdf** `island_threads` element under `ode::solver`
    + description: Number of threads to use for "islands" of disconnected models.
    + type: int
    + default: 0
    + required: 0
    + [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. **physics.sdf** `thread_position_correction` element under `ode::solver`
    + description: Flag to use threading to speed up position correction computation.
    + type: bool
    + default: 0
    + required: 0
    + [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. **state.sdf** allow `light` tags within `insertions` element
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/325)

1. **surface.sdf** `category_bitmask` element
    + description: Bitmask for category of collision filtering.
      Collision happens if `((category1 & collision2) | (category2 & collision1))` is not zero.
      If not specified, the category_bitmask should be interpreted as being the same as collide_bitmask.
    + type: unsigned int
    + default: 65535
    + required: 0
    + [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/318)

1. **world.sdf** `wind` element
    + description: The wind tag specifies the type and properties of the wind.
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **world.sdf** `wind::linear_velocity` element
    + description: Linear velocity of the wind.
    + type: vector3
    + default: "0 0 0"
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)
