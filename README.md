# ez-Wheel SWD® ROS2 Controllers
## Overview

This package has been tested on ROS2 Galactic. It contains ROS2 nodes to control motors powered by the [ez-Wheel](https://www.ez-wheel.com) Safety Wheel Drive (SWD®) technology.

| <img src="https://www.ez-wheel.com/storage/image-product/visuels-swd-core-2-0-0.png" width="45%"></img> | <img src="https://www.ez-wheel.com/storage/image-product/roue-electrique-swd-150-2-0-0-0.png" width="45%"></img> | <img src="https://www.ez-wheel.com/storage/image-product/starterkit-ez-wheel-web-0-0-0.png" width="45%"></img>       |
| ------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| [SWD® Core](https://www.ez-wheel.com/en/safety-gear-motor) <br />Safety gear motor                      | [SWD® 150](https://www.ez-wheel.com/en/swd-150-safety-wheel-drive) <br />Safety wheel drive                      | [SWD® StarterKit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) <br />Development kit for AGV and AMR |

## Installation

This package has been tested on **x64_86** and ~~**arm64**~~ machines.
Pre-built packages are available for ROS2 Galactic on Ubuntu 20.04 (for **x64_86** and ~~**arm64**~~).

### Prerequisites

- A SWD® based wheel
- `SWD firmware` (**`>= 1.0.0`**)
- Ubuntu 20.04
- ROS2 Galactic
- `swd-services` (**`>= 0.2.0`**)

### Ubuntu

In order to install `swd_ros2_controllers`, you need to add the ez-Wheel repository to `/etc/apt/sources.list`.

```shell
echo "deb http://packages.ez-wheel.com:8081/apt-repo focal main" | sudo tee -a /etc/apt/sources.list
```

Then download and add the GPG key using following command:

```shell
wget -qO - http://packages.ez-wheel.com:8081/archive.key | sudo apt-key add -
```

Now, you should be able to install the `ros-galactic-swd-ros2-controllers` package using `apt`:

```shell
sudo apt update && sudo apt install ros-galactic-swd-ros2-controllers
```

### Compiling from source

To compile the package, make sure you have added the ez-Wheel repository to your `/etc/apt/sources.list` as specified above.
Then you need to install `swd-services` using:

```shell
sudo apt-get update && sudo apt install swd-services
```

In the following instructions, replace `<rosdistro>` with the name of your ROS2 distro (e.g., `galactic`).

```shell
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone https://github.com/ezWheelSAS/swd_ros2_controllers.git
cd ..
colcon build
source ~/ros2_ws/install/setup.bash
```

## Usage

The package comes with preconfigured `.launch` files which can be started using the `ros2 launch` command:
- `swd_diff_drive_controller_launch.py`: sample configuration for the [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) differential drive robot. To use it, run the following command:

```shell
ros2 launch swd_ros2_controllers swd_diff_drive_controller.launch.py
```

You can always use the nodes with the `ros2 run` command, the minimum required parameters are:

- `swd_ros2_controllers`:

```shell
ros2 run swd_ros2_controllers swd_diff_drive_controller --ros-args -p baseline_m:=0.485
```
The corresponding D-Bus services have to be started in order to use the nodes.
Example for the [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) differential drive robot:
* ezw-dbus-user-session.service (dbus-launch > /tmp/SYSTEMCTL_dbus.id) [**OPTIONAL**]
> export $(cat /tmp/SYSTEMCTL_dbus.id) [**OPTIONAL**]

> export LD_LIBRARY_PATH=/opt/ezw/usr/lib
* ezw-swd-left.service (/opt/ezw/usr/bin/ezw-smc-service /opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini)
* ezw-swd-right.service (/opt/ezw/usr/bin/ezw-smc-service /opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini)

Example of configuration files for [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) differential drive robot:

swd_left_config.ini
```
# SMC Drive service config file
contextId = 12
nodeId = 4
coreNodeId = 6
coreNodeIsMaster = true ; Slave:false Master:true
canDevice = can0
dbusNamespace = swd_left

HWConfigurationEntry = SWD_CORE
HWConfigurationFile = /opt/ezw/data/configuration.json

CANOpenEDSFile = /opt/ezw/usr/etc/ezw-canopen-dico/swd_core.eds
```

swd_right_config.ini
```
# SMC Drive service config file
contextId = 12
nodeId = 5
coreNodeId = 7
coreNodeIsMaster = true # Slave:false Master:true
canDevice = can0
dbusNamespace = swd_right

HWConfigurationEntry = SWD_CORE
HWConfigurationFile = /opt/ezw/data/configuration.json

CANOpenEDSFile = /opt/ezw/usr/etc/ezw-canopen-dico/swd_core.eds
```

configuration.json
```
[
   {
      "name": "SWD_CORE",
      "nbStepRevolutionElec": 6,
      "nbPolePair": 5,
      "reduction": 14.0,
      "diameter": 125.0
   }
]
```
## The `swd_diff_drive_controller` node

This controller drives two ez-Wheel SWD® wheels as a differential-drive robot.

### Parameters

- `baseline_m` of type **`double`**: The distance (in meters) between the 2 wheels (**mandatory**).
- `left_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the left motor (default /opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini) (**read only**).
- `right_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the right motor (default /opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini) (**read only**).
- `pub_freq_hz` of type **`int`**: Frequency (in Hz) of published odometry and TFs (default `20`).
- `watchdog_receive_ms` of type **`int`**: The delay (in milliseconds) before stopping the wheels if no command is received (default `500`) (**read only**).
- `base_frame` of type **`string`**: Frame ID for the moving platform, used in odometry and TFs (default `'base_link'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `odom_frame` of type **`string`**: Frame ID for the `odom` fixed frame used in odometry and TFs (default `'odom'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `publish_odom` of type **`bool`**: Publish odometry messages (default `true`).
- `publish_tf` of type **`bool`**: Publish TF messages (default `true`).
- `publish_safety_functions` of type **`bool`**: Publish **`swd_ros2_controllers::msg::SafetyFunctions`** message (default `true`).
- `motor_max_speed_rpm` of type **`int`**: Maximum allowed motor speed (in RPM), if a target speed of one of the motor is above this limit, the controller will limit the speed of the two motor without changing the robot's trajectory (default `1050`).
- `motor_max_safety_limited_speed_rpm` of type **`int`**: Motor safety limited speed (SLS) (in RPM), if an SLS signal is detected (from a security LiDAR for example), the motor will be limited internally to the configured SLS limit. The ROS2 controller uses this value to limit the target speed sent to the motor in the SLS case (default `490`).
- `have_backward_sls` of type **`bool`**: Specifies if the robot have a backward SLS signal, coming for example from a back-facing security LiDAR. If an SLS signal is available for backward movements, set this to `true` to take it into account. Otherwise, set the parameter to `false`, this will limit all backward movements to the selected `safety_limited_speed_rpm` (default `false`).
- `left_encoder_relative_error` of type **`double`**: Relative error for left wheel encoder, used to calculate variances and propagate them to calculate the uncertainties in the odometry message. Each encoder acquisition **`DIFF_LEFT_ENCODER`** is modeled as: **`DIFF_LEFT_ENCODER +/- abs(left_encoder_relative_error * DIFF_LEFT_ENCODER)`** (default `0.2` corresponding to 20% of error).
- `right_encoder_relative_error` of type **`double`**: Relative error for right wheel encoder, used to calculate variances and propagate them to calculate the uncertainties in the odometry message. Each encoder acquisition **`DIFF_RIGHT_ENCODER`** is modeled as: **`DIFF_RIGHT_ENCODER +/- abs(right_encoder_relative_error * DIFF_RIGHT_ENCODER)`** (default `0.2` corresponding to 20% of error).

### Subscribed Topics

- `/cmd_vel` of type **`geometry_msgs/msg/Twist`**: Target linear and angular velocities.
- `/set_speed` of type **`geometry_msgs/msg/Point`**: Target speeds in rad/s for left (`Point.x`) and right (`Point.y`) wheels.
- `/soft_brake` of type **`std_msgs/msg/Bool`**: Activate or release the soft brake, send `false` to release the brake, or `true` to activate it.

### Published Topics

- `/odom` of type **`nav_msgs/msg/Odometry`**: Odometry message based on wheels encoders, containing the pose and velocity of the robot with their's associated uncertainties. Unless disabled by the `publish_tf` parameter, TFs with the same information are also published.
- `/safety` of type **`swd_ros2_controllers/msg/SafetyFunctions`**: Safety messages communicated by the wheels via CANOpen, the message includes information about Safe Torque Off (STO), Safety Limited Speed (SLS), Safe Direction Indication (forward/backward) (SDI+/-), and Safe Brake Control (SBC).

## Custom message types

### The `swd_ros2_controllers::msg::SafetyFunctions` message

This message provides information about CiA 402-4 CANopen safety drive functions.
True if the safety drive function is enabled.

```
std_msgs/Header header
bool safe_torque_off                        # Safe Torque Off (STO)
bool safe_brake_control                     # Safe Brake Control (SBC)
bool safety_limited_speed                   # Safety Limited Speed (SLS)
bool safe_direction_indication_forward      # Safe Direction Indication (positive)
bool safe_direction_indication_backward     # Safe Direction Indication (negative)
```
The main safe drive function is the STO whereby the immediately torque-off on the motor may be accompanied by an SBC command to close the brakes. 
The SLS functions cause the drive to decelerate (if required) and monitor whether the velocity is held within the defined limits.
The functions SDIp and SDIn enable the motor movement only in the corresponding (positive or negative) direction. 

## Support

For any questions, please [open a GitHub issue](https://github.com/ezWheelSAS/swd_ros_controllers/issues).

## About ez-Wheel®

**ez-Wheel®** is an innovative company founded in 2009 and located in Angoulême, France.
The ez-Wheel company has developed the first industrial drive wheel, integrating electric traction motor, embedded electronics and rechargeable batteries.

This revolutionary solution, which quickly turns any manually handled platform into an electrically assisted one.
Our solutions have been adopted by hundreds of end-users to improve productivity and prevent work accidents caused by manual handling.
Our products are used in a variety of applications, in fields of Automotive, Factory logistics, Warehouses, Food processing, Hospitals and Pharmaceutical industries.

The new SWD® product family targets industrial robotics applications, like Autonomous Mobile Robots (AMRs) and Automatic Guided Vehicles (AGVs).
It provides a unique solution for safety critical systems, which provides safety features according to the **ISO 3691-4** standard.

The [ez-Wheel®](https://www.ez-wheel.com) company has developed a unique know-how in embedded electronics, including safety critical systems, applied to battery powered electric traction.
