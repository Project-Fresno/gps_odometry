#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos


class IMU_ODOM(Node):
    def __init__(self):
        super().__init__("imu_odom")

        self.gnss_pose_subscriber = self.create_subscription(
            PoseStamped, "/gnss_pose", self.gnss_pose_callback
        )

        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.origin_set = False
        self.origin_pose = [0, 0]
        self.origin_heading = 0

    def gnss_pose_callback(self, pose_msg):
        if not self.origin_set:
            self.origin_pose = pose_msg.pose.pose

            Q = pose_msg.pose.orientation
            self.origin_heading = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])[2]

            self.origin_set = True

        del_lat = pose_msg.pose.pose.x - self.origin_pose.x
        del_lon = pose_msg.pose.pose.y - self.origin_pose.y

        Q = pose_msg.pose.orientation
        curr_heading = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])[2]
        theta = curr_heading - self.origin_heading
        x, y, z, w = quaternion_from_euler(0, 0, theta)

        del_x = del_lat * cos(theta) + del_lon * sin(theta)
        del_y = -del_lat * sin(theta) + del_lon * cos(theta)

        odom = Odometry()
        odom.child_frame_id = "base_link"
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = del_x
        odom.pose.pose.position.y = del_y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w

        self.odom_publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = del_x
        t.transform.translation.y = del_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    imu_odom_publisher = IMU_ODOM()
    rclpy.spin(imu_odom_publisher)

    imu_odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
