#! /usr/bin/env python

import os
import sys

import rospy

from ed_object_models.sdf_tools import spawn_sdf_from_yaml

if __name__ == "__main__":
    my_args = rospy.myargv(sys.argv)
    # Initialize ROS node
    rospy.init_node("gazebo_object_spawner")

    yaml_path = my_args[1]
    if not os.path.isabs(yaml_path):
        yaml_path = os.path.join(os.path.curdir, yaml_path)

    spawn_sdf_from_yaml(yaml_path)
