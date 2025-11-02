# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# import numpy as np
# import struct

# class RawToRawEx(Node):
#     def __init__(self):
#         super().__init__('raw_to_raw_ex')
#         self.sub = self.create_subscription(PointCloud2, '/sensing/lidar/top/pointcloud_raw', self.callback, 10)
#         self.pub = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw_ex', 10)

#     def callback(self, msg):
#         points = []
#         for i in range(msg.width):
#             offset = i * msg.point_step
#             # x, y, z, intensity, ring, time
#             x = struct.unpack_from('f', msg.data, offset + 0)[0]
#             y = struct.unpack_from('f', msg.data, offset + 4)[0]
#             z = struct.unpack_from('f', msg.data, offset + 8)[0]
#             intensity = struct.unpack_from('f', msg.data, offset + 12)[0]
#             ring = struct.unpack_from('I', msg.data, offset + 16)[0]
#             time = struct.unpack_from('f', msg.data, offset + 18)[0]

#             # 补充扩展字段
#             distance = np.sqrt(x**2 + y**2 + z**2)
#             azimuth = np.arctan2(y, x)
#             elevation = np.arctan2(z, np.sqrt(x**2 + y**2))
#             return_type = 0  # 默认为0

#             # intensity转uint8
#             intensity_uint8 = int(intensity) if intensity >= 0 else 0
#             intensity_uint8 = max(0, min(intensity_uint8, 255))

#             # channel转uint16
#             channel_uint16 = ring & 0xFFFF

#             # time_stamp转uint32（可按实际需求缩放）
#             # 例如将秒转为微秒后取uint32
#             time_stamp_uint32 = int(time * 1e6) & 0xFFFFFFFF

#             # 构造新点
#             point = struct.pack('<fffBBHfffI',
#                 x, y, z,
#                 intensity_uint8,      # uint8
#                 return_type,          # uint8
#                 channel_uint16,       # uint16
#                 azimuth,              # float32
#                 elevation,            # float32
#                 distance,             # float32
#                 time_stamp_uint32     # uint32
#             )
#             points.append(point)

#         # 构造新的 PointCloud2 消息
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#             PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
#             PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
#             PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
#             PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
#             PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
#             PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
#             PointField(name='time_stamp', offset=28, datatype=PointField.UINT32, count=1),
#         ]
#         data = b''.join(points)
#         new_msg = PointCloud2()
#         new_msg.header = msg.header
#         new_msg.height = 1
#         new_msg.width = msg.width
#         new_msg.fields = fields
#         new_msg.is_bigendian = False
#         new_msg.point_step = 32  # 每个点字节数
#         new_msg.row_step = new_msg.point_step * new_msg.width
#         new_msg.data = data
#         new_msg.is_dense = True

#         self.pub.publish(new_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = RawToRawEx()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import struct

class RawToRawEx(Node):
    def __init__(self):
        super().__init__('raw_to_raw_ex')

        # 话题列表ss
        self.topics = [
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/top/pointcloud_raw_ex'),
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/left/pointcloud_raw_ex'),
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/right/pointcloud_raw_ex'),
        ]

        # 为每个话题创建订阅和发布器
        self.subs = []
        self.pubs = {}
        for sub_topic, pub_topic in self.topics:
            sub = self.create_subscription(PointCloud2, sub_topic, self.make_callback(pub_topic), 10)
            self.subs.append(sub)
            self.pubs[pub_topic] = self.create_publisher(PointCloud2, pub_topic, 10)

        self.timer = self.create_timer(0.1, self.timer_callback) 
            
    def timer_callback(self):
        self.get_logger().info('定时器回调被调用')
        
    def make_callback(self, pub_topic):
        def callback(msg):
            points = []
            for i in range(msg.width):
                offset = i * msg.point_step
                # x, y, z, intensity, ring, time
                x = struct.unpack_from('f', msg.data, offset + 0)[0]
                y = struct.unpack_from('f', msg.data, offset + 4)[0]
                z = struct.unpack_from('f', msg.data, offset + 8)[0]
                intensity = struct.unpack_from('f', msg.data, offset + 12)[0]
                ring = struct.unpack_from('I', msg.data, offset + 16)[0]
                time = struct.unpack_from('f', msg.data, offset + 18)[0]

                # 扩展字段
                distance = np.sqrt(x**2 + y**2 + z**2)
                azimuth = np.arctan2(y, x)
                elevation = np.arctan2(z, np.sqrt(x**2 + y**2))
                return_type = 0

                intensity_uint8 = int(intensity) if intensity >= 0 else 0
                intensity_uint8 = max(0, min(intensity_uint8, 255))
                channel_uint16 = ring & 0xFFFF
                time_stamp_uint32 = int(time * 1e6) & 0xFFFFFFFF

                point = struct.pack('<fffBBHfffI',
                    x, y, z,
                    intensity_uint8,
                    return_type,
                    channel_uint16,
                    azimuth,
                    elevation,
                    distance,
                    time_stamp_uint32
                )
                points.append(point)

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
                PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
                PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
                PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
                PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
                PointField(name='time_stamp', offset=28, datatype=PointField.UINT32, count=1),
            ]
            data = b''.join(points)
            new_msg = PointCloud2()
            new_msg.header = msg.header
            new_msg.height = 1
            new_msg.width = msg.width
            new_msg.fields = fields
            new_msg.is_bigendian = False
            new_msg.point_step = 32
            new_msg.row_step = new_msg.point_step * new_msg.width
            new_msg.data = data
            new_msg.is_dense = True

            self.pubs[pub_topic].publish(new_msg)
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = RawToRawEx()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
