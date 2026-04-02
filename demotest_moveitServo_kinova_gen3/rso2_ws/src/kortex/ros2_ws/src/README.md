# kortex 
[![pipeline status](https://gitlab.rwu.de/prj-iki-ros2/robots/kortex/badges/jazzy/pipeline.svg)](https://gitlab.rwu.de/prj-iki-ros2/robots/kortex/-/commits/jazzy)

This repo is used to control the Kinova Gen3 Kortex robot arm. As the support and ROS2 drivers are suboptimal, some changes are had to be made.

***Note: as multiple packages had to be changed in the kortex repo a fork has been made. The for currently lies in [My Github repo](https://github.com/DavidG-Develop/ros2_kortex). If you want to continue work on it either fork it or let me know so i can add you as a collaborator.***

Problem descriptions are also found below :arrow_down:.

## Simulation

### Build

The CI already builds the images but you can also build them loacally by using:

```bash
docker-compose -f docker/compose_sim.yaml build
```

### Run

```bash
docker-compose -f docker/compose_sim.yaml up
```

## Real robot

Connect the Kortex to your PC using the Lan interface. Unlock the safety power switch and power on the robot by holding the button on the rear of the base for 5 seconds. The robot is fully powered on when the gripper opens, closes and opens again. move the arm to a safe location before using the B (Home) or A (Retract) buttons on the joystick. The button needs to be held and not only pressed.

### Build

The CI already builds the images but you can also build them loacally by using:

```bash
docker-compose -f docker/compose.yaml build
```

### Run

The arm can be controlled from a single PC setup or a double PC setup. Double PC setup is reccomended due to with increasing resources used on the pc that is controlling the arm the control cycle speed falls which the arm does not like. Also with to much resources in use the HardwareInterface crashes.

#### Single pc setup

Connect lan from the robot arm directly to the pc (or using a switch). Set up your lan settings to the same network as the robot arm -> 192.168.1.XXX with mask 255.255.255.0. than run:

```bash
docker-compose -f docker/compose.yaml up
```

#### Double pc setup

Connect both pcs and the arm to the same switch. Set up both pc lan settings to the same network as the robot arm -> 192.168.1.XXX with mask 255.255.255.0. than run:

```bash
docker-compose -f docker/compose.yaml up kortex kortex_vision
```
on the pc that only controls the arm.

And:
```bash
docker-compose -f docker/compose.yaml up kortex_rviz kortex_moveit
```
on the pc that is running high computation stuff.

If the second pc is not a laptop/does not have a integrated screen, it is recommended to set the static ip for the kortex network to be the same as the hardcoded ROS_DOMAIN_ID in the docker compose file so it is easier to find where to ssh to.

#### Double pc setup with jetson Xavier

If using the arm with the current jetson Xavier than it can also be used without it being connected to a screen as it is already configured.

Once the arm, jetson Xavier and your pc are connected to the switch and you set up your network, on your pc run:

```bash
ssh barrob@192.168.1.145
cd ~/ros2_ws/src/prj-iki-ros2/robots/kortex
docker-compose -f docker/compose.yaml up kortex
```
and:
```bash
docker-compose -f docker/compose.yaml up kortex_rviz kortex_moveit kortex_vision
```
***Note: In this setup the vision is running on the main PC due to the jetson Xavier not being powerfull enough. Main plan is to have one pc "the arm pc" running arm + vision with the other one doing rest of computation.***



## Problems list

### Gripper problems

The whole setup is made to be used plug and play style with the robotiq_2f_85 gripper and not the robotiq_2f_140 gripper. The 140 gripper, although it is compatible and on the 1st try seems to work, it doesent close fully. The URDF of the gripper defines the fully closed position as ~0.7 while the real gripper requires ~0.8 to fully close. ( A github ticket is opened to fix this but the devs are very slow if there are any... https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/79 )

The 2 current solutins are as following:

1. Modify the URDF:

Currently to achieve the desired gripper closing position, the closing position is set to 0.7929 with the actuated finger, in the urdf, is set to move with 1x speed while the other finger mimics with 0.875x speed. This in theory works but provides with a slightly offseted urdf gripping position which is not perfect. 

If anyone has extra time, a rewrite of the robotiq_2f_140 gripper is suggested. after it has been rewritten and tested a PR to the original ros2_robotiq_gripper repo should be made. It is always nice to give back to the community (60k€ arm :rofl:)

2. Modify the hardware interface:

As the arm and the gripper run of the same KortexMultiInterfaceHardware class, a simple change of the gripper read and write functions should do the trick. In the hardware interface an additional parameter was added (gripper_model) which is read on initialization and used to modify the gripper read and write functions to match the gripper model. Only if gripper is set to robotiq_2f_140 the gripper read and write functions are modified with the scaled values.

What this does it makes the ros side drive in the range 0 - 0.7 while the gripper is actually driven with 0 - 0.8. This means all the data in the urdf stays the same (except for the added gripper_model parameter) and the close in the moveit config should that also be set back to 0.7 (stock value).


***NOTE: We currently use the solution Nr 2 which means gripper close is 0.7!***

### Hardware interface crashing

<pre>
<span style="color: red">[ros2_control_node-1] [ERROR] [KortexMultiInterfaceHardware]: Runtime error: timeout detected: BaseCyclicClient::Refresh</span>
<span style="color: yellow">[ros2_control_node-1] [WARN] [controller_manager]: Overrrun detected! The controller manager missed its desired rate of 1000Hz. The loop took 3020.002688ms (missed cycles : 3021).</span>
</pre>

The problem with the hardware interface is that due to higher load (actually almost any load) the KortexMultiInterfaceHardware starts blocking and the arm misses all cycle updates. After the arm misses all cycle updates for a couple of seconds the following error shows up [ KortexMultiInterfaceHardware ]: Runtime error: timeout detected: BaseCyclicClient::Refresh and the arm is no longer movable (it also forces ros2 control into a crash). This is also a known issue but nothing is being done :) (https://github.com/Kinovarobotics/ros2_kortex/issues/248)

Unfortunately the problem arrises even with simple tasks like subscribing to the data of the arm camera... The current workaround is using lowered resolution and compressed images to reduce the load. Here the arm + moveit + rviz + vision node with lowered resolution and compressed images works but as soon as something more powerfull is ran besides it (like GPD) it crashes again.

***Current possible solution*** (Not yet tested) run it using multiple pcs -> run the arm on something like a jetson (this arm must be controller with 1000Hz so having a realtime kernel is also a +), on jetson also run the vision (hopefully with normal resolution) and moveit -> run all other parts needed, like gpd, on another PC.

Feel free to look into the source code of the Hardware interface, drivers and vision to try to resolve this issue without the need for multiple pcs :) . Some effort has already been done to turn the kortex_vision into a component node so it can do 0 copy with the image proc nodes but that is also still in progress (see [ros2_kortex_vision](./ros2_kortex_vision/)).