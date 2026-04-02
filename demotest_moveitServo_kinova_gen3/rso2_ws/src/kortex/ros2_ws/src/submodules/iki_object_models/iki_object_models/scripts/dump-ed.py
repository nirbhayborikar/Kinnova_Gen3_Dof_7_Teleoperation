#! /usr/bin/env python

from ed_msgs.srv import SimpleQuery
import rospy
import PyKDL as kdl
import sys

if __name__ == "__main__":

    if len(sys.argv) <= 1:
        print("Please provide robot name")
        sys.exit(1)

    robot_name = sys.argv[1]

    rospy.init_node("dump_ed")

    rospy.wait_for_service('/{}/ed/simple_query'.format(robot_name))
    try:
        ed_query = rospy.ServiceProxy('/{}/ed/simple_query'.format(robot_name), SimpleQuery)
        res = ed_query()

        for e in res.entities:
            if not e.has_shape:
                continue

            if e.id.startswith(robot_name):
                continue

            q_msg = e.pose.orientation
            q = kdl.Rotation.Quaternion(q_msg.x, q_msg.y, q_msg.z, q_msg.w)

            yaw = q.GetRPY()[2]  # 0:.2f

            print("- id: {}".format(e.id))
            print("  type: {}".format(e.type))

            if abs(yaw) > 0.001:
                print("  pose: {{ x: {}, y: {}, z: {}, Z: {:.3f} }}".format(e.pose.position.x, e.pose.position.y, e.pose.position.z, yaw))
            else:
                print("  pose: {{ x: {}, y: {}, z: {} }}".format(e.pose.position.x, e.pose.position.y, e.pose.position.z))

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
