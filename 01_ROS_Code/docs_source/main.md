## Nomenclature used in the Dynamixel Motors
The ReConBot has been divided in three main groups in order to use it when controlling or planning. These groups are named as:

* reconbot_controller
* r_arm_reconbot_controller
* l_arm_reconbot_controller

Each of this groups are compounded by a group of links and of course, a group of Dynamixel MX-64AT as it is shown:

reconbot_controller | r_arm_reconbot_controller | l_arm_reconbot_controller
:------------------:|:-------------------------:|:------------------------:
ID:001 = joint_1    | ID:001                    | ID:004
ID:002 = joint_2    | ID:002                    | ID:005
ID:003 = joint_3    | ID:003                    | ID:006
ID:004 = joint_4    | -                         | -
ID:005 = joint_5    | -                         | -
ID:006 = joint_6    | -                         | -

\note
  ID:00x represents the ID assigned internally and it is used in order to identify each of the motors when interfacing with the **/ttty/USB0** port.

\todo Continue writing the manual...


# Structure of the ReConBot package for ROS on Linux
---

In this section a step by step guide is given aimed to get running the ReConBot core on Linux.
## What I need for controlling the reconbot?
 For controlling the ReConBot are neccesary the following components:
 * a PC running Linux 14.04 LTS. This PC must have ROS Indigo installed.
 * a PC running MATLAB in order to use all the toolboxes developed for the ReConBot at IGM.
 * an USB2Dynamixel interface or programmer. This programmer could be directed connected to the PC via USB port or using a wifi connection, in this case an extra component will be necessary.
 * Power adapter for the Dynamixel motors.

## Get the ReConBot running on Linux

The ReConBot is controlled using the **ReConBot** package which can be downloaded  from the IGM GitLab repository. Clone the repository into the *src* folder of the catkin workspace. In a Debian based machine this could be done typing on the Linux console:

```bash
cd ~/catkin_ws/src
git clone
```
now the catkin workspace has to be built again:

```bash
cd ..
catkin_make
```
\note Here it is supposed that a catkin workspace has been configured in the Linux computer in advance.

Once the catkin workspace is built, it is possible to launch the ReConBot controllers using the main launch file. The main launch file comes with three different modes the controllers can be configured for.

* *full_mode:* Select this mode if the ReConBot is going to be controlled using all the motors (7 motors). The default value of this parameter in the launch files is *false*. Hence, it should be switched to *true*.

```bash
roslaunch reconbot_control main.launch full_mode:=true
```

\todo add a new section for the ReConBot package that was created for MATLAB.
\todo Start thinking about the transformation services needed in order to transform the coordinate systems from the controllers reference system to the MATLAB code and vice versa.
