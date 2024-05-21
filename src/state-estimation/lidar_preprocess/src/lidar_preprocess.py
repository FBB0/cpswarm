#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2 
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointField
import numpy as np
# import laser_geometry.laser_geometry as lg

# TODO: find the orientation of scan of obstacles, filter them out
class Lidar_Preprocess:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.pc2_pub = rospy.Publisher('mirte/converted_pc2', PointCloud2, queue_size=1)

    def scan_callback(self, msg):
        filtered_points = self.filter_scan(msg)
        self.publish_filtered_points(filtered_points, msg.header)

    def filter_scan(self, msg):
        filtered_points = []
        for i, range_value in enumerate(msg.ranges):
            point_orientation_rad = msg.angle_min + msg.angle_increment * i
            if not self.is_point_occluded(point_orientation_rad):
                point_local_x = range_value * np.cos(point_orientation_rad)
                point_local_y = range_value * np.sin(point_orientation_rad)
                filtered_points.append([point_local_x, point_local_y, 0.0])
                # filtered_points.append([point_local_x, point_local_y])

        return filtered_points

    def is_point_occluded(self, angle_rad):
        return False
            
        

    def publish_filtered_points(self, filtered_points, scan_header):

        pc2_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pc2_msg = pc2.create_cloud(header=scan_header, fields=pc2_fields, points=filtered_points)
        # pc2_msg.is_dense = True

        self.pc2_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_preprocess')
    lidar_preprocess = Lidar_Preprocess()
    rospy.spin()







