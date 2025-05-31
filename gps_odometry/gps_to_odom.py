#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

import math
import utm
import numpy as np


class GPS_TO_ODOM(Node):
    def __init__(self):
        super().__init__("gps_to_odom")

        self.gnss_subscriber = self.create_subscription(
            NavSatFix, "/gnss", self.gnss_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Quaternion, "/filter/quaternion", self.imu_callback, 10
        )

        self.odom_publisher = self.create_publisher(Odometry, "/odometry/gps", 10)
        self.initial_covariance = None

        self.rotation = None

        self.initial_yaw = 0.0

        self.i = 0
        self.j = 0

        self.initial_utm = None

    def gnss_callback(self, gnss):
        latitude = gnss.latitude
        longitude = gnss.longitude

        odom = Odometry()
        pose = Pose()

        if self.i == 1:
            if self.j == 0:
                self.initial_utm = utm.from_latlon(latitude, longitude)
                self.initial_covariance = (
                    self.rotation
                    @ np.array(
                        [
                            [gnss.position_covariance[0], 0.0],
                            [0.0, gnss.position_covariance[4]],
                        ]
                    )
                    @ self.rotation.T
                )
                self.j = 1

            self.utm = utm.from_latlon(latitude, longitude)

            coordinates = self.pose()
            x = coordinates[0]
            y = coordinates[1]

            print(coordinates)

            pose.position.x = x
            pose.position.y = y
            odom.pose.pose = pose

            odom.pose.covariance[0] = (
                self.rotation
                @ np.array(
                    [
                        [gnss.position_covariance[0], 0.0],
                        [0.0, gnss.position_covariance[4]],
                    ]
                )
                @ self.rotation.T
                + self.initial_covariance
            )[0, 0]
            odom.pose.covariance[7] = (
                self.rotation
                @ np.array(
                    [
                        [gnss.position_covariance[0], 0.0],
                        [0.0, gnss.position_covariance[4]],
                    ]
                )
                @ self.rotation.T
                + self.initial_covariance
            )[1, 1]

            odom.child_frame_id = "base_link"
            odom.header.frame_id = "odom"
            self.odom_publisher.publish(odom)

    def imu_callback(self, quaternion):
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = w * w + x * x - y * y - z * z

        if self.i == 0:
            self.initial_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.rotation = np.array(
                [
                    [np.cos(self.initial_yaw), -np.sin(self.initial_yaw)],
                    [np.sin(self.initial_yaw), np.cos(self.initial_yaw)],
                ]
            )
            self.i = 1

    def pose(self):
        initial_coordinates = self.rotation @ np.array(
            [[self.initial_utm[0]], [self.initial_utm[1]]]
        )

        temp_coordinates = self.rotation @ np.array([[self.utm[0]], [self.utm[1]]])

        x = (temp_coordinates - initial_coordinates)[0, 0]
        y = (temp_coordinates - initial_coordinates)[1, 0]

        coordinates = [x, y]
        return coordinates


def main(args=None):
    rclpy.init(args=args)

    gps_odom_publisher = GPS_TO_ODOM()
    rclpy.spin(gps_odom_publisher)

    gps_odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
