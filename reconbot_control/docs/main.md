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
