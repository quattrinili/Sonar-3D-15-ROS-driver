import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from sonar3d.api.inspect_sonar_data import parse_rip1_packet, decode_protobuf_packet, rangeImageToXYZ
from sonar3d.api.interface_sonar_api import set_acoustics, describe_response, enable_multicast
import socket
import struct
import numpy as np
import math

class TimerNode(Node):

    # Multicast group and port used by the Sonar 3D-15
    MULTICAST_GROUP = '224.0.0.96'
    PORT = 4747

    # The maximum possible packet size for Sonar 3D-15 data
    BUFFER_SIZE = 65535

    def __init__(self):
        super().__init__('timer_node')
        
        # Declare parameters
        self.declare_parameter('IP', '192.168.194.96')# '192.168.194.96' is the fallback ip, to change this, edit the launchfile.
        self.declare_parameter('speed_of_sound', 1491)    # setting this takes ~20s

        self.declare_parameter('realtime', True)    # setting this takes ~20s

        self.sonar_speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.realtime = self.get_parameter('realtime').get_parameter_value().bool_value

        # Create a timer that calls the timer_callback every sample_time seconds 
        if self.realtime:
            self.sonar_ip = self.get_parameter('IP').get_parameter_value().string_value
            sample_time = 0.01          # sample time in seconds
            self.create_timer(sample_time, self.timer_callback)
            self.get_logger().info(f'Timer Node initialized with {1/sample_time} Hz')
            # Enable the acoustics on the sonar
            resp = set_acoustics(self.sonar_ip, True)
            self.get_logger().info(f'Enabling acoustics response: {describe_response(self.sonar_ip, resp)}')

            resp = enable_multicast(self.sonar_ip)
            self.get_logger().info(f'Enabling multicast response: {describe_response(self.sonar_ip, resp)}')

            # Set up a UDP socket with multicast membership
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', self.PORT))

            group = socket.inet_aton(self.MULTICAST_GROUP)
            mreq = struct.pack('4sL', group, socket.INADDR_ANY)
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

            self.get_logger().info(f"Listening for Sonar 3D-15 RIP1 packets on {self.MULTICAST_GROUP}:{self.PORT}...")

            if self.sonar_ip != "":
                self.get_logger().info(f"Filtering packets from IP: {self.sonar_ip}")
        else:
            from std_msgs.msg import UInt8MultiArray
            self.sonar_ip = ""  # No IP filtering in non-realtime mode
            self.raw_subscriber_ = self.create_subscription(
                UInt8MultiArray,
                'tube1/sonar_raw',
                self.raw_data_callback,
                10)

        # Create a publisher that publishes the point cloud data
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'sonar_point_cloud', 10)
        self.image_publisher_ = self.create_publisher(Image, 'sonar_range_image', 10)
        self.intensity_image_publisher_ = self.create_publisher(Image, 'sonar_intensity_image', 10)

        # Store latest intensity data for correlation with RangeImage
        self.latest_intensity_data = None
        self.latest_intensity_dims = None

        


    def timer_callback(self):

        data, addr = self.sock.recvfrom(self.BUFFER_SIZE)

        # If SONAR_IP is configured, and this doesn't match the known Sonar IP, skip it.
        if not (addr[0] == self.sonar_ip or addr[0] == '192.168.194.96'):
            self.get_logger().info(f"Received packet from {addr[0]}. Data was received from an IP that does not match the declared SONAR_IP ({self.sonar_ip}), so the packet will be skipped.")
            return
        
        self.process_packet(data)

    def raw_data_callback(self, msg):
        self.process_packet(bytes(msg.data))

    def process_packet(self, data):

        payload = parse_rip1_packet(data)
        if payload is None:
            self.get_logger().warning("Parsed payload is None, skipping packet.")
            return
        # Decode the Protobuf message
        result = decode_protobuf_packet(payload)
        if not result:
            self.get_logger().warning("Decoding Protobuf packet failed, skipping packet.")
            return

        msg_type, msg_obj = result

        if msg_type == 'RangeImage':
            # Convert the RangeImage message to voxel data
            voxels = rangeImageToXYZ(msg_obj)
            seq_id = msg_obj.header.sequence_id

            # Use the latest intensity data received
            intensity_2d = None
            if self.latest_intensity_data is not None and self.latest_intensity_dims == (msg_obj.height, msg_obj.width):
                intensity_2d = self.latest_intensity_data.reshape((msg_obj.height, msg_obj.width))
            
            # Debug log
            if intensity_2d is not None:
                self.get_logger().info(f"RangeImage seq={seq_id} with intensity - Min: {intensity_2d.min()}, Max: {intensity_2d.max()}, Mean: {intensity_2d.mean():.2f}")
            else:
                self.get_logger().warning(f"RangeImage seq={seq_id} - no intensity data available")

            # Create a PointCloud2 message with intensity field
            header = Header()
            header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
            header.frame_id = 'sonar_frame'

            # Build PointCloud2 with x, y, z, intensity fields
            pts_data = bytearray()
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 16  # 4 fields * 4 bytes each

            for py in range(msg_obj.height):
                for px in range(msg_obj.width):
                    pixel_idx = py * msg_obj.width + px
                    pixel_value = msg_obj.image_pixel_data[pixel_idx]
                    
                    # Skip zero pixels (no data)
                    if pixel_value == 0:
                        continue
                    
                    # Find the corresponding voxel
                    max_px = msg_obj.width - 1
                    max_py = msg_obj.height - 1
                    fov_h = math.radians(msg_obj.fov_horizontal)
                    fov_v = math.radians(msg_obj.fov_vertical)
                    
                    yaw_rad = (px / max_px) * fov_h - fov_h / 2
                    pitch_rad = (py / max_py) * fov_v - fov_v / 2
                    dist = pixel_value * msg_obj.image_pixel_scale
                    
                    x = dist * math.cos(pitch_rad) * math.cos(yaw_rad)
                    y = dist * math.cos(pitch_rad) * math.sin(yaw_rad)
                    z = -dist * math.sin(pitch_rad)
                    
                    # Get intensity value from 2D array at same pixel location
                    intensity = 0.0
                    if intensity_2d is not None:
                        intensity = float(intensity_2d[py, px]) / 255.0  # Normalize to 0-1
                    
                    # Pack as float32
                    pts_data += struct.pack('<ffff', x, y, z, intensity)

            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1
            cloud_msg.width = len(voxels)
            cloud_msg.fields = fields
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = point_step
            cloud_msg.row_step = point_step * len(voxels)
            cloud_msg.data = bytes(pts_data)
            cloud_msg.is_dense = True

            # Publish the PointCloud2 message
            self.pointcloud_publisher_.publish(cloud_msg)

            # Publish the raw range image
            img_msg = Image()
            img_msg.header = header
            img_msg.height = msg_obj.height
            img_msg.width = msg_obj.width
            img_msg.encoding = '32FC1'
            img_msg.is_bigendian = False
            img_msg.step = msg_obj.width * 4
            range_image = (np.array(msg_obj.image_pixel_data, dtype=np.uint32) * msg_obj.image_pixel_scale).astype(np.float32)
            img_msg.data = range_image.tobytes()
            self.image_publisher_.publish(img_msg)

        elif msg_type == 'BitmapImageGreyscale8':
            # Extract intensity values
            # BitmapImageGreyscale8.image_pixel_data is raw bytes, use frombuffer
            intensity_data = np.frombuffer(msg_obj.image_pixel_data, dtype=np.uint8)
            seq_id = msg_obj.header.sequence_id
            
            # Store as latest intensity data for use with next RangeImage
            self.latest_intensity_data = intensity_data
            self.latest_intensity_dims = (msg_obj.height, msg_obj.width)
            
            # Log intensity stats
            self.get_logger().debug(f"BitmapImageGreyscale8 Seq ID: {seq_id}, Stats - Min: {intensity_data.min()}, Max: {intensity_data.max()}, Mean: {intensity_data.mean():.2f}")
            
            # Publish intensity as Image message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'sonar_frame'
            
            img_msg = Image()
            img_msg.header = header
            img_msg.height = msg_obj.height
            img_msg.width = msg_obj.width
            img_msg.encoding = 'mono8'
            img_msg.is_bigendian = False
            img_msg.step = msg_obj.width
            img_msg.data = intensity_data.tobytes()
            self.intensity_image_publisher_.publish(img_msg)

        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()