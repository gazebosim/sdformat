# Manipulating a SDF Panda Robot Arm in Isaac Sim with USD

Download the panda model from Fuel (make sure you have [Ignition Fuel Tools installed](https://ignitionrobotics.org/api/fuel_tools/5.0/install.html))

```bash
ign fuel download -u "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/panda with ignition position controller model"
```

Copy the empty world [`empty.sdf`](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo6/examples/worlds/empty.sdf) to a file called `panda.sdf` and add these lines inside the `<world>` tag:

```xml
<include>
  <pose>0 0 0 0 0 0</pose>
  <name>panda</name>
  <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/panda with ignition position controller model</uri>
</include>
```

Run the converter (you must build sdformat first):

```bash
usdConverter panda.sdf panda.usd
```

Open IsaacSim and load the `panda.usd` file.

Add to the `panda` prim the following:

 - Create -> Isaac -> ROS -> Clock
 - Create -> Isaac -> ROS -> Joint State
    - Set *targetPrims* to `/fuel/panda`
 - Create -> Isaac -> ROS -> Pose tree
    - Set *articulationPrim* to `/fuel/panda`

Now run the simulation.

## Move the arm (option 1):

Open a new terminal and source ROS:

```bash
source /opt/ros/<rosdistro>/setup.bash
```

Create a file, call it `move_arm.py` and copy the following script:

```python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time

rospy.init_node("test_rosbridge", anonymous=True)

pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
joint_state = JointState()

joint_state.name = [
    "elbow",
    "shoulder_lift",
    "shoulder_pan",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

num_joints = len(joint_state.name)

joint_state.position = np.array([0.0] * num_joints)
default_joints = [0.0, 1.57, 0.707, 0.707, 0, 3.1416]

# limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement)
max_joints = np.array(default_joints) + 0.707
min_joints = np.array(default_joints) - 0.07

# position control the robot to wiggle around each joint
time_start = time.time()
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    joint_state.position = np.sin(time.time() - time_start) * (max_joints - min_joints) * 0.5 + default_joints
    pub.publish(joint_state)
    rate.sleep()
```

Now you can move the robot typing:

```bash
python3 move_robot.py
```

## Move the arm (option 2)

Inside `~/.local/share/ov/pkg/isaac_sim-2021.2.1/` you might find a ROS workspace under the folder `ros_workspace`.
Source your ROS distro.

Compile the code:

```bash
source /opt/ros/<rosdistro>/setup.bash
cd ~/.local/share/ov/pkg/isaac_sim-2021.2.1/ros_workspace
catkin_make_isolated
source devel_isolated/setup.bash
```

Run the Moveit example:

```bash
roslaunch isaac_moveit franka_isaac_execution.launch
```

If you want to learn more about how to move the arm using MoveIt, you should follow [these instruccions](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros_moveit.html#running-moveit).
