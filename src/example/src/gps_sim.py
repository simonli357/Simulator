#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

def publish_waypoints():
    rospy.init_node('waypoint_publisher', anonymous=True)
    pub = rospy.Publisher('/gps', PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    waypoints = [(1, 1), (2, 2), (3, 3)]
    num_waypoints = len(waypoints)
    current_index = 0

    while not rospy.is_shutdown():
        waypoint = waypoints[current_index]
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = waypoint[0]
        pose_msg.pose.pose.position.y = waypoint[1]
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        pub.publish(pose_msg)
        rospy.loginfo("Publishing waypoint: ({}, {})".format(waypoint[0], waypoint[1]))
        rate.sleep()

        # Move to the next waypoint
        current_index = (current_index + 1) % num_waypoints

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
