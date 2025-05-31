#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos
import numpy as np
from utm import from_latlon


class IMU_ODOM(Node):
    def __init__(self):
        super().__init__("imu_odom")

        self.gnss_pose_subscriber = self.create_subscription(
            PoseStamped, "/gnss_pose", self.gnss_pose_callback, 10
        )

        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.origin_set = False
        self.origin_utm = np.array([0.0, 0.0])
        self.origin_heading = 0.0

    def gnss_pose_callback(self, pose_msg):
        # Extract latitude and longitude (assuming x=lat, y=lon)
        lat = pose_msg.pose.position.x
        lon = pose_msg.pose.position.y

        if not self.origin_set:
            # Set origin in UTM coordinates
            utm_coords = from_latlon(lat, lon)
            self.origin_utm = np.array(
                [utm_coords[0], utm_coords[1]]
            )  # [easting, northing]

            # Extract yaw from quaternion and keep east-based orientation
            Q = pose_msg.pose.orientation
            roll, pitch, yaw_east = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])

            self.origin_heading = yaw_east  # Keep east-based heading as is

            self.origin_set = True
            self.get_logger().info(
                f"Origin set: UTM({self.origin_utm[0]:.2f}, {self.origin_utm[1]:.2f}), Heading: {self.origin_heading:.3f} rad"
            )
            return

        # Convert current position to UTM
        utm_coords = from_latlon(lat, lon)
        current_utm = np.array([utm_coords[0], utm_coords[1]])  # [easting, northing]

        # Calculate UTM displacement from origin
        del_easting = current_utm[0] - self.origin_utm[0]
        del_northing = current_utm[1] - self.origin_utm[1]

        # Extract current heading and keep east-based orientation
        Q = pose_msg.pose.orientation
        roll, pitch, yaw_east = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
        current_heading = yaw_east  # Keep east-based heading as is

        # Transform from UTM (east/north) to robot local frame (x/y) in east-based frame
        # In east-based frame: 0° = east, 90° = north
        # Robot frame: x = forward (in direction of heading), y = left
        del_x = del_easting * cos(self.origin_heading) + del_northing * sin(
            self.origin_heading
        )
        del_y = -del_easting * sin(self.origin_heading) + del_northing * cos(
            self.origin_heading
        )

        # Calculate relative heading
        theta = current_heading - self.origin_heading

        # Create quaternion for relative orientation
        x, y, z, w = quaternion_from_euler(0, 0, theta)

        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = del_x
        odom.pose.pose.position.y = del_y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w

        self.odom_publisher.publish(odom)

        # Broadcast transform
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
