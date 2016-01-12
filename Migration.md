# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete sdformat code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## SDFormat 3.x to 4.x

### Additions

1. **New SDF 1.6**
   + All sdf files were copied from 1.5 except IMU (see modifications)

1. **New SDF: Wind**
   + In world element: the wind tag specifies the type and properties of the
     wind
   + In link and model elements: enable_wind tag to indicate if it is affected
     by the wind.

1. **SDF Model state: new scale element**
   + In model state element: scale tag for the 3 dimensions of the model

### Modifications

1. **Boost pointers and boost::function**
   + All boost pointers, boost::function in the public API have been replaced
     by their std:: equivalents (C++11 standard)

1. **SDF IMU: noise**
   + TODO

1. **Lump:: prefix in link names**
   + Changed to \_fixed_joint_lump__ to avoid confusion with scoped names

1. **SDF Physics: gravity and magnetic_field moved to World**
   + In physics element: gravity and magnetic_field tags have been moved
     from Physics to World element.
