set (sdfs
  actor.sdf 
  camera.sdf
  collision.sdf 
  contact.sdf
  gazebo.sdf 
  geometry.sdf 
  gripper.sdf
  gui.sdf
  inertial.sdf 
  joint.sdf 
  light.sdf 
  link.sdf 
  model.sdf 
  physics.sdf 
  plugin.sdf 
  projector.sdf
  ray.sdf
  rfidtag.sdf
  rfid.sdf
  road.sdf
  scene.sdf 
  sensor.sdf 
  state.sdf
  surface.sdf 
  visual.sdf 
  world.sdf
)

sdf_install_sdf_files("${sdfs}" "1.0")
