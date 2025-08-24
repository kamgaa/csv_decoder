import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

import csv
import math
import os
import time

class CSVDecoderNode(Node):
    def __init__(self):
        super().__init__('csv_decoder')
        self.cf01_pub = self.create_publisher(Pose, '/cf01/goal', 10)
        self.cf02_pub = self.create_publisher(Pose, '/cf02/goal', 10)
        
        self.trigger_sub = self.create_subscription(Int32, '/csv_trigger', self.trigger_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz


        #self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # 좌표 기준점 및 스케일링
        self.cf01_index = -1
        self.cf02_index = -1

        self.cf01_arrived = False
        self.cf02_arrived = False

        self.cf01_pose = None
        self.cf02_pose = None


        # 기준점과 스케일 (SpaFrancorchamps.csv 용)
        #self.x_ref = 1061
        #self.y_ref = 1436
        #self.scale = 0.001
        
        # CSV 파일 로드
        package_share = get_package_share_directory('csv_decoder')
        #csv_path = os.path.join(package_share, 'config', 'SpaFrancorchamps.csv')
        #csv_path = os.path.join(package_share, 'config', 'figure8.csv')
        #csv_path = os.path.join(package_share, 'config', 'figure0.csv')
        lead_path = os.path.join(package_share, 'config', 'figure8_lead.csv')
        follower_path = os.path.join(package_share, 'config', 'figure8_foll.csv')
        #with open(csv_path, newline='', encoding='utf-8-sig') as csvfile:
        #    reader = csv.reader(csvfile)
        #    self.data = [(float(row[0]), float(row[1]),float(row[3])) for row in reader]
        with open(lead_path, newline='', encoding='utf-8-sig') as f1:
            read_leader = csv.reader(f1)
            self.data_lead = [(float(row[0]), float(row[1]), float(row[3])) for row in read_leader]
        with open(follower_path, newline='', encoding='utf-8-sig') as f2:
            read_follower = csv.reader(f2)
            self.data_foll = [(float(row[0]), float(row[1]), float(row[3])) for row in read_follower]

    def trigger_callback(self, msg):
        if msg.data == 1111:
            self.get_logger().info('cf01 triggered')
            self.cf01_index = 0
            self.cf01_arrived = False
        elif msg.data == 1100:
            self.get_logger().info('cf02 triggered')
            self.cf02_index = 0
            
            self.cf02_arrived = False    
        
   
    def timer_callback(self):
        #elapsed = time.time() - self.start_time
        # cf01 도착 확인 및 index 초기화
        if self.cf01_arrived and not self.cf02_arrived:
            self.get_logger().info('Leader arrived first, stopping follower')
            self.cf02_arrived = True      
        # cf01 퍼블리시
        if not self.cf01_arrived and self.cf01_index >= 0 and self.cf01_index < len(self.data_lead):
            x_raw, y_raw, yaw = self.data_lead[self.cf01_index]
            msg = Pose()
            msg.position.x = x_raw 
            msg.position.y = y_raw 
            msg.position.z = 1.0
            msg.orientation.z = yaw
            self.cf01_pub.publish(msg)
            self.cf01_index += 1
        elif not self.cf01_arrived and self.cf01_index >= 0 and self.cf01_index < len(self.data_lead):           # leader 경로 끝에 도달
            self.cf01_arrived = True

        # cf02 퍼블리시
        if not self.cf02_arrived and self.cf02_index >= 0 and self.cf02_index < len(self.data_foll):
            x_raw, y_raw, yaw = self.data_foll[self.cf02_index]
            msg = Pose()
            msg.position.x = x_raw 
            msg.position.y = y_raw
            msg.position.z = 1.0
            msg.orientation.z = yaw
            self.cf02_pub.publish(msg)
            self.cf02_index += 1
        elif not self.cf02_arrived and self.cf02_index >= 0 and self.cf02_index < len(self.data_foll): # follower 경로 끝에 도달
            self.cf02_arrived = True



def main(args=None):
    rclpy.init(args=args)
    node = CSVDecoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

