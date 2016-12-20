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
but with improved human-readability.

## SDF protocol 1.5 to 1.6

### Additions

1. **heightmap_shape.sdf** `sampling` element
    + description: Samples per heightmap datum.
      For rasterized heightmaps, this indicates the number of samples to take per pixel.
      Using a lower value, e.g. 1, will generally improve the performance
      of the heightmap but lower the heightmap quality.
    + type: unsigned int
    + default: 2
    + required: 0
    + [pull request 293](https://bitbucket.org/osrf/sdformat/pull-requests/293)

1. **link.sdf** `enable_wind` element
    + description: If true, the link is affected by the wind
    + type: bool
    + default: false
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

1. **model.sdf** `enable_wind` element
    + description: If set to true, all links in the model
      will be affected by the wind.
      Can be overriden by the link wind property.
    + type: bool
    + default: false
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

1. **model_state.sdf** `scale` element
    + description: Scale for the 3 dimensions of the model.
    + type: vector3
    + default: "1 1 1"
    + required: 0
    + [pull request 246](https://bitbucket.org/osrf/sdformat/pull-requests/246)

1. **physics.sdf** `friction_model` element
    + description: Name of ODE friction model to use. Valid values include:
        + pyramid_model: (default) friction forces limited in two directions
          in proportion to normal force.
        + box_model: friction forces limited to constant in two directions.
        + cone_model: friction force magnitude limited in proportion to normal force.
          See [gazebo pull request 1522](https://bitbucket.org/osrf/gazebo/pull-request/1522)
          (merged in [gazebo 8c05ad64967c](https://bitbucket.org/osrf/gazebo/commits/8c05ad64967c))
          for the implementation of this feature.
    + type: string
    + default: "pyramid_model"
    + required: 0
    + [pull request 294](https://bitbucket.org/osrf/sdformat/pull-requests/294)

1. **world.sdf** `wind` element
    + description: The wind tag specifies the type and properties of the wind.
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

1. **world.sdf** `wind::linear_velocity` element
    + description: Linear velocity of the wind.
    + type: vector3
    + default: "0 0 0"
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

### Modifications

1. `gravity` and `magnetic_field` elements are moved
    from `physics` to `world`
    + [pull request 247](https://bitbucket.org/osrf/sdformat/pull-requests/247)
    + [gazebo pull request 2090](https://bitbucket.org/osrf/gazebo/pull-requests/2090)

1. A new style for representing the noise properties of an `imu` was implemented
   in [pull request 199](https://bitbucket.org/osrf/sdformat/pull-requests/199)
   for sdf 1.5 and the old style was declared as deprecated.
   The old style has been removed from sdf 1.6 with the conversion script
   updating to the new style.
    + [pull request 199](https://bitbucket.org/osrf/sdformat/pull-requests/199)
    + [pull request 243](https://bitbucket.org/osrf/sdformat/pull-requests/243)
    + [pull request 244](https://bitbucket.org/osrf/sdformat/pull-requests/244)

