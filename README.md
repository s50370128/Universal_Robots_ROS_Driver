[![Build badge](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/workflows/Industrial%20CI%20pipeline/badge.svg?branch=master&event=push)](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/actions)

# Universal_Robots_ROS_Driver

## Acknowledgment
This driver is forked from the [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).

## Contents
This repository contains the new **ur_robot_driver** and a couple of helper packages, such as:

  * **controller_stopper**: A small external tool that stops and restarts ros-controllers based on
    the robot's state. This can be helpful when the robot is in a state where it won't accept
    commands sent from ROS.
  * **ur_calibration**: Package around extracting and converting a robot's factory calibration
    information to make it usable by the robot_description.
  * **ur_robot_driver**: The actual driver package.

Please see the external [feature list](ur_robot_driver/doc/features.md) for a listing of all features supported by this driver.

## Requirements
This driver requires a system setup with ROS. It is recommended to use **Ubuntu 18.04 with ROS
melodic**, however using Ubuntu 20.04 with ROS noetic should also work.

To make sure that robot control isn't affected by system latencies, it is highly recommended to use
a real-time kernel with the system. See the [real-time setup guide](ur_robot_driver/doc/real_time.md)
on information how to set this up.

## Building

**Note:** The driver consists of a [C++
library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) that abstracts the
robot's interfaces and a ROS driver on top of that. As the library can be built without ROS support,
it is not a catkin package and therefore requires a different treatment when being built inside the
workspace. See The alternative build method below if you'd like to build the library from source.

If you don't want to build the library from source, it is available as a binary package through the
ROS distribution of ROS melodic and noetic. It will be installed automatically if you
follow the steps below. If you'd like to also build the library from source, please follow the steps
explained in the [next section](#alternative-all-source-build).

```bash
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone -b dual_arm https://github.com/s50370128/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

## Setting up a UR robot for ur_robot_driver
### Prepare the robot
For using the *ur_robot_driver* with a real robot you need to install the
**externalcontrol-1.0.5.urcap** which can be found inside the **resources** folder of this driver.

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is
necessary.

For installing the necessary URCap and creating a program, please see the individual tutorials on
how to [setup an e-Series robot](ur_robot_driver/doc/install_urcap_e_series.md).

To setup the tool communication on an e-Series robot, please consider the
[tool communication setup guide](ur_robot_driver/doc/setup_tool_communication.md).

### Prepare the ROS PC
For a more elaborate tutorial on how to get started, please see the
[usage example](ur_robot_driver/doc/usage_example.md).

#### Extract calibration information
Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended
to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script:

    $ roslaunch ur_calibration calibration_correction.launch \
      robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As
`target_filename` provide an absolute path where the result will be saved to.

We recommend keeping calibrations for all robots in your organization in a common package. See the
[package's documentation](ur_calibration/README.md) for details.

#### Quick start
To actually start the robot driver use one of the existing launch files

    $ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101

where **<robot_type>** is one of *ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e*. Note that in this example we
load the calibration parameters for the robot "ur10_example".

If you calibrated your robot before, pass that calibration to the launch file:

    $ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 \
      kinematics_config:=$(rospack find ur_calibration)/etc/ur10_example_calibration.yaml

If the parameters in that file don't match the ones reported from the robot, the driver will output
an error during startup, but will remain usable.

Once the robot driver is started, load the [previously generated program](#prepare-the-robot) on the
robot panel that will start the *External Control* program node and execute it. From that moment on
the robot is fully functional. You can make use of the *Pause* function or even *Stop* (:stop_button:) the
program. Simply press the *Play* button (:arrow_forward:) again and the ROS driver will reconnect.

Inside the ROS terminal running the driver you should see the output `Robot ready to receive control commands.`


To control the robot using ROS, use the action server on

```bash
/scaled_pos_joint_traj_controller/follow_joint_trajectory
```

Use this with any client interface such as [MoveIt!](https://moveit.ros.org/) or simply the
`rqt_joint_trajectory_controller` gui:

```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

You may need to install rqt_joint_trajectory_controller by running: 
```
sudo apt install ros-<ROS-DISTRO>-rqt-joint-trajectory-controller
```

### Replacing the robot description

In a real-world scenario you will want to replace the robot description with a description
containing the whole scene where the robot is acting in. For this, all the bringup launchfiles offer
the argument `robot_description_file` that should point to a launchfile loading the robot
description.

While the `load_urXXX.launch` files from the [ur_description](http://wiki.ros.org/ur_description)
package contain a lot of arguments to change the robot model, this driver only forwards the
`kinematics_config` parameter file. For further adaption please create your own `load_urXXX.launch`
file that fits your application and pass this to the `urXXX_bringup.launch` files from this package.

If you prefer decoupling loading the robot description and starting the driver, you can start the
`ur_control.launch` launchfile directly after the `robot_description` has been uploaded to the
parameter server.

## Troubleshooting

This section will cover some previously raised issues.

### I started everything, but I cannot control the robot.
The `External Control` program node from the URCap is not running on the robot. Make sure to create
a program containing this node on the robot and start it. Inside the ROS terminal you should see
the output `Robot ready to receive control commands.`

**Note:** When interacting with the teach pendant, or sending other primary programs to the robot, the
program will be stopped. On the ROS terminal you will see an output `Connection to robot dropped,
waiting for new connection`. In those cases, restart program execution (e.g. by pressing the play
button on the TP, or calling `rosservice call /ur_hardware_interface/dashboard/play` as explained [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/33) and [here](ur_robot_driver/doc/ROS_INTERFACE.md#ur_robot_driver_node)).

In general, make sure you've completed the following tasks:

1. Install and setup the `External Control` URCap as explained
   [above](#setting-up-a-ur-robot-for-ur_robot_driver) (also setup the IP address **of the ROS
   pc** inside the URCap's installation.)
2. Start the driver handing the IP **of the robot** as explained in the
   [quickstart](#quick-start)
3. Load and start the previously generated program on the TP.

### When starting the program on the TP, I get an error "The connection to the remote PC could not be established"
Make sure, the IP address setup is correct, as described in the setup guides ([e-Series robots](ur_robot_driver/doc/install_urcap_e_series.md))

**Note:** This error can also show up, when the ROS driver is not running.

### When starting the program on the TP, I get a `C207A0` error.
**Note:** With the current driver version this issue can only happen when the fieldbus is enabled
*after* the ROS driver has been started. Otherwise you will run into [#204](../../issues/204) when starting the driver
with an enabled EtherNet/IP fieldbus.

Most probably, the EtherNet/IP fieldbus is enabled in the robot's installation. If your setup includes an
Ethernet/IP fieldbus (note: EtherNet/IP != ethernet), make sure that it is
connected properly. In the Ethernet/IP fieldbus Installation screen
(e-series: *Installation > Fieldbus > EtherNet/IP*, CB3: *Installation > EtherNet/IP*) you can select the action that is being
executed upon a loss of EtherNet/IP Scanner connection. If you select "None",
save installation and program, then no exception is raised when no connection
to the fieldbus scanner can be established (note: This is only to get the
`External Control` running. You probably want to make sure that a connection to
the fieldbus scanner can indeed be made). If you don't use EtherNet/IP
fieldbusses at all, you can disable it in the same installation screen. 

### When starting the driver, it crashes with `Variable 'speed_slider_mask' is currently controlled by another RTDE client`
Probably, you are running into [#204](../../issues/204). Currently, this driver cannot be used together with an enabled
EtherNet/IP fieldbus. Disable EtherNet/IP to workaround this error. [#204](../../issues/204) contains a guide how to do
this.


### I cannot get a realtime kernel running together with an NVIDIA graphics card
This is a known issue and unfortunately we don't have a solution for this. The Nvidia kernel module
seems to not compile with every kernel. We recommend to use a multi-machine ROS setup in this
situation where a realtime-system is running the robot driver and a separate machine is performing
the computations requiring the graphics card.

### Why can't the driver use the extracted calibration info on startup?
This is mainly because parameters are loaded onto the parameter server before any nodes are started.

The `robot_description` concept inside ROS is not designed to be changed while a system is running.
Consumers of the urdf/`robot_description` (in terms of other ROS nodes) will not update the model
they have been loading initially. While technically the `robot_description` parameter could be altered during runtime
and any node that is started *afterwards* would see the updated model, this would lead to an inconsistent
application state (as some nodes will use the old model, while others use the updated one). In other words: It's not the driver that needs/benefits from this calibrated urdf, it's the rest of the ROS application and that will only see it *if* the calibrated version is present on the parameter server *before* nodes are started.

Additionally: If the calibration stored on the ROS side doesn't match the one of the robot controller, there's a good chance there is a reason for this and it
would be better to make updating it a conscious decision by a human (as the driver would not know *when* updating the model would be convenient or safe). Having to run the calibration
extraction/transformation as a separate step makes this possible and doesn't hide this step from the
end user.

### Can this driver be used inside a combined hardware interface?
Yes, this is possible. However, if used inside a [combined HW
interface](http://wiki.ros.org/combined_robot_hw) we recommend to enable [non-blocking read
functinality](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/ROS_INTERFACE.md#non_blocking_read-default-false).

### I sent raw script code to the robot but it is not executed
On the e-Series the robot has to be in [remote control
mode](ur-robot-driver/README.md#remote-control-mode) to accept script code from an external source.
This has to be switched from the Teach-Pendant.

### Using the dashboard doesn't work
On the e-Series the robot has to be in [remote control
mode](ur-robot-driver/README.md#remote-control-mode) to accept certain calls on the dashboard server.
See [Available dashboard
commands](https://www.universal-robots.com/articles/ur-articles/dashboard-server-cb-series-port-29999/)
for details.

### Passthrough controllers: The robot does not fully reach trajectory points even though I have specified the path tolerance to be 0
If you are using a control modes that forwards trajectories to the robot, currently the path tolerance is ignored. The corresponding interface on the robot and client-library level exists in the form of a "blend radius", but is not utilized by this ROS driver. For more information see this [issue](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/352).

### Can I use the Cartesian controllers together with MoveIt!?
Not directly, no. MoveIt! plans a Cartesian path and then creates a joint trajectory out of that for
execution, as the common interface to robot drivers in ROS is the
[FollowJointTrajectory](http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html)
action.

For supporting Cartesian controllers inside MoveIt! changes would have to be made to MoveIt! itself.
