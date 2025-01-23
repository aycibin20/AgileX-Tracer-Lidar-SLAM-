#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

# Global değişkenler
front_scan = None
rear_scan = None

def front_lidar_callback(msg):
    global front_scan
    front_scan = msg

def rear_lidar_callback(msg):
    global rear_scan
    rear_scan = msg

def merge_lidar_data():
    rospy.init_node('merge_lidar_node')

    rospy.Subscriber('/front_lidar', LaserScan, front_lidar_callback)
    rospy.Subscriber('/rear_lidar', LaserScan, rear_lidar_callback)

    pub = rospy.Publisher('/merged_lidar', LaserScan, queue_size=10)

    rate = rospy.Rate(20)  # Default: 20 Hz

    while not rospy.is_shutdown():
        if front_scan is None or rear_scan is None:
            rospy.logwarn("Waiting for lidar data...")
            continue

        merged_scan = LaserScan()
        merged_scan.header.stamp = rospy.Time.now()
        merged_scan.header.frame_id = 'base_link'

        # Genel tarama özelliklerini ayarla
        merged_scan.angle_min = front_scan.angle_min
        merged_scan.angle_max = front_scan.angle_min + len(front_scan.ranges) * front_scan.angle_increment
        merged_scan.angle_increment = front_scan.angle_increment
        merged_scan.time_increment = front_scan.time_increment
        merged_scan.scan_time = front_scan.scan_time
        merged_scan.range_min = min(front_scan.range_min, rear_scan.range_min)
        merged_scan.range_max = max(front_scan.range_max, rear_scan.range_max)

        # Ön ve arka lidar verilerini hazırla (NaN ve Inf değerlerini işleyerek)
        front_ranges = [r if not np.isinf(r) and not np.isnan(r) else front_scan.range_max for r in front_scan.ranges]
        rear_ranges = [r if not np.isinf(r) and not np.isnan(r) else rear_scan.range_max for r in rear_scan.ranges[::-1]]

        # Ranges listesini birleştirirken en küçük mesafeyi seç
        merged_ranges = [
            min(front, rear) for front, rear in zip(front_ranges, rear_ranges)
        ]

        # Birleşik veriyi ekle
        merged_scan.ranges = merged_ranges
        merged_scan.intensities = [
            max(front_int, rear_int) for front_int, rear_int in zip(front_scan.intensities, rear_scan.intensities[::-1])
        ]

        # Birleşik veriyi yayınla
        pub.publish(merged_scan)
        rate.sleep()

if __name__ == "__main__":
    try:
        merge_lidar_data()
    except rospy.ROSInterruptException:
        pass
