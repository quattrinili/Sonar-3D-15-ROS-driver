#!/usr/bin/env python3

# ------------------------------------------------------------------------------
# Developer and Contact:
# Marios Xanthidis
# Research Scientist @ SINTEF Ocean
# Email: marios.xanthidis@sintef.no
#
# License:
# This software is released under a permissive free-use license.
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files, to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the software,
# subject to the following conditions:
# - The above contact and attribution notice shall be included in all copies or
#   substantial portions of the software.
# - This software is provided "as is", without warranty of any kind, expressed
#   or implied.
#
# Aknowledgements:
# - Supported by the Research Council of Norway (EchoNav: NO-359447)
# - Filtering and name conventions adapted from Alberto Quattrini Li @ Dartmouth
#   His repository for ROS1 integration of the Sonar 3D-15 can be found in:
#   https://github.com/quattrinili/Sonar-3D-15-api-example/tree/ros1
# ------------------------------------------------------------------------------

"""
Info:

Convert Sonar 3D-15 .sonar files to ROS2 bagfiles (ROS2 Humble version).
To save them to a bag, run `ros2 bag record` in a separate terminal.

Usage:
    1. Start recording in one terminal:
        ros2 bag record -o <output_bag_dir> /sonar3d/range_image /sonar3d/point_cloud
    2. In another terminal, run:
        python3 sonar_to_bag.py --file <sonar_file.sonar> --realtime-factor 1.0

Requirements:
    - Tested with ROS2 Humble and Jazzy
    - numpy, protobuf, sonar_3d_15_protocol_pb2.py
    - sensor_msgs, std_msgs, builtin_interfaces
    - cv_bridge (for ROS2)
"""

import os
import struct
import zlib
import math
import numpy as np
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge

SONAR3D_FRAME = "sonar3d"
SONAR_RANGE_IMAGE_TOPIC = f'/{SONAR3D_FRAME}/range_image'
SONAR_POINT_CLOUD_TOPIC = f'/{SONAR3D_FRAME}/point_cloud'
YEAR_CHECK = 2023

from sonar_3d_15_protocol_pb2 import Packet, BitmapImageGreyscale8, RangeImage

def parse_rip1_packet(data: bytes):
    if len(data) < 13:
        return None
    magic = data[:4]
    if magic != b'RIP1':
        return None
    total_length = struct.unpack('<I', data[4:8])[0]
    if len(data) < total_length:
        return None
    payload = data[8: total_length - 4]
    crc_received = struct.unpack('<I', data[total_length - 4: total_length])[0]
    crc_calculated = zlib.crc32(data[: total_length - 4]) & 0xffffffff
    if crc_calculated != crc_received:
        return None
    return payload

def decode_protobuf_packet(payload: bytes):
    packet = Packet()
    try:
        packet.ParseFromString(payload)
    except Exception as e:
        print(f"Protobuf parse error: {e}")
        return None
    any_msg = packet.msg
    if not any_msg.IsInitialized():
        return None
    bmp = BitmapImageGreyscale8()
    if any_msg.Unpack(bmp):
        return ("BitmapImageGreyscale8", bmp)
    rng = RangeImage()
    if any_msg.Unpack(rng):
        return ("RangeImage", rng)
    return ("Unknown", any_msg)

def publishImage(bmpImg):
    image = np.zeros((bmpImg.height, bmpImg.width, 1), dtype=np.uint8)
    for y in range(bmpImg.height-1, -1, -1):
        for x in range(bmpImg.width):
            idx = y * bmpImg.width + x
            if idx < len(bmpImg.image_pixel_data):
                image[bmpImg.height-y-1][x] = bmpImg.image_pixel_data[idx]
    return image

def rangeImageToXYZ(ri):
    max_pixel_x = ri.width - 1
    max_pixel_y = ri.height - 1
    fov_h = math.radians(ri.fov_horizontal)
    fov_v = math.radians(ri.fov_vertical)
    points = []
    for pixel_x in range(ri.width):
        for pixel_y in range(ri.height):
            idx = pixel_y * ri.width + pixel_x
            if idx >= len(ri.image_pixel_data):
                continue
            pixel_value = ri.image_pixel_data[idx]
            if pixel_value == 0:
                continue
            yaw_rad = (pixel_x / max_pixel_x) * fov_h - fov_h / 2
            pitch_rad = (pixel_y / max_pixel_y) * fov_v - fov_v / 2
            distance_meters = pixel_value * ri.image_pixel_scale
            x = distance_meters * math.cos(pitch_rad) * math.cos(yaw_rad)
            y = -distance_meters * math.cos(pitch_rad) * math.sin(yaw_rad)
            z = distance_meters * math.sin(pitch_rad)
            points.append([x, y, z, yaw_rad, pitch_rad, distance_meters])
    return points

def make_ros2_time(stamp_float):
    sec = int(stamp_float)
    nanosec = int((stamp_float - sec) * 1e9)
    t = TimeMsg()
    t.sec = sec
    t.nanosec = nanosec
    return t

def handle_packet(pkt, bridge):
    payload = parse_rip1_packet(pkt)
    if payload is None:
        return None
    result = decode_protobuf_packet(payload)
    if not result:
        return None
    msg_type, msg_obj = result
    if hasattr(msg_obj, "header") and hasattr(msg_obj.header, "timestamp"):
        dt = msg_obj.header.timestamp.ToDatetime()
        if dt.year < YEAR_CHECK:
            return None
        stamp = dt.timestamp()
    else:
        return None
    if msg_type == "BitmapImageGreyscale8":
        image = publishImage(msg_obj)
        ros_image_msg = bridge.cv2_to_imgmsg(image, "mono8")
        ros_image_msg.header.stamp = make_ros2_time(stamp)
        ros_image_msg.header.frame_id = SONAR3D_FRAME
        return ("image", ros_image_msg, stamp)
    elif msg_type == "RangeImage":
        points = rangeImageToXYZ(msg_obj)
        if not points:
            return None
        # Define PointFields
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="yaw", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="pitch", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="distance", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        # Create PointCloud2
        pc2 = PointCloud2()
        pc2.header.stamp = make_ros2_time(stamp)
        pc2.header.frame_id = SONAR3D_FRAME
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 24  # 6 floats * 4 bytes
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        arr = np.array(points, dtype=np.float32)
        pc2.data = arr.tobytes()
        return ("point_cloud", pc2, stamp)
    else:
        return None

def check_file_existence(filename):
    try:
        with open(filename, 'rb') as f:
            pass
    except FileNotFoundError:
        print(f"File not found: {filename}")
        exit(1)

class SonarPublisher(Node):
    def __init__(self, sonar_packets, bridge, topic_image, topic_pointcloud, realtime_factor=1.0):
        super().__init__('sonar_publisher')
        self.image_pub = self.create_publisher(Image, topic_image, 10)
        self.pc_pub = self.create_publisher(PointCloud2, topic_pointcloud, 10)
        self.sonar_packets = sonar_packets
        self.bridge = bridge
        self.realtime_factor = realtime_factor

    def publish_all(self):
        count = 0
        prev_stamp = None
        for pkt in self.sonar_packets:
            result = handle_packet(pkt, self.bridge)
            if not result:
                continue
            msg_type, msg_obj, stamp = result

            # Sleep to synchronize publishing to original timestamps
            if prev_stamp is not None:
                dt = (stamp - prev_stamp) / self.realtime_factor
                if dt > 0:
                    time.sleep(dt)
            prev_stamp = stamp

            if msg_type == "image":
                self.image_pub.publish(msg_obj)
                self.get_logger().info(f"Published image at {stamp}")
            elif msg_type == "point_cloud":
                self.pc_pub.publish(msg_obj)
                self.get_logger().info(f"Published point cloud at {stamp}")
            rclpy.spin_once(self, timeout_sec=0.02)  # allow message to be sent
            count += 1
        self.get_logger().info(f"Done publishing {count} messages.")

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Convert Sonar 3D-15 .sonar file to ROS2 messages (Humble compatible) and publish them synchronously to their timestamps."
    )
    parser.add_argument(
        "--file",
        type=str,
        required=True,
        help="Sonar .sonar file to convert."
    )
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0=real time, 2.0=2x speed, 0.5=half speed)"
    )
    args = parser.parse_args()
    check_file_existence(args.file)

    # Read and split packets from file
    with open(args.file, 'rb') as f:
        content = f.read()
    packets = content.split(b'RIP1')
    packets = [b'RIP1'+pkt for pkt in packets if pkt]

    rclpy.init(args=None)
    bridge = CvBridge()
    node = SonarPublisher(
        sonar_packets=packets,
        bridge=bridge,
        topic_image=SONAR_RANGE_IMAGE_TOPIC,
        topic_pointcloud=SONAR_POINT_CLOUD_TOPIC,
        realtime_factor=args.realtime_factor
    )
    try:
        node.publish_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("All messages published. You may now stop ros2 bag record if you wish.")

if __name__ == "__main__":
    main()