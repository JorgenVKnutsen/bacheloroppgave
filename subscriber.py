#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int64
from geometry_msgs.msg import TwistWithCovarianceStamped
import pandas as pd
from datetime import datetime
import time


class Subscriber(Node):
   def __init__(self):
      super().__init__("data_subscriber")

      #Setting up all export lists
      self.u1_list_ = []
      self.u2_list_ = []
      self.u3_list_ = []
      self.velocity_list_ = []
      self.angle_list_ = []
      self.time_stamp_ = []

      #Declearing all required variables
      self.velocity_reading_ = 0
      self.angular_reading_ = 0
      self.current_u1 = 0
      self.current_u2 = 0
      self.current_u3 = 0

      #Setting up timestamps
      self.time_start_ = time.time()
      self.time_now_ = 0


      self.get_logger().info("Subscriber started")

   
      self.throttle_sub_ = self.create_subscription(Int64, 'lw_portena_throttle', self.update_throttle_input, 10)
      self.brake_sub_ = self.create_subscription(Int64, 'lw_portena_brake', self.update_brake_input, 10)
      self.steering_sub_ = self.create_subscription(Int64, 'lw_portena_steering', self.update_steering_input, 10)



      #subscribing to actual data to be used. The callback will update the lists with all current data 
      self.state_subscriber = self.create_subscription(TwistWithCovarianceStamped, 'vectornav/velocity_body', self.update_data, 10)


   #Functions to update the data for dataframe. This is done for the data to be matched to current data
   def update_throttle_input(self, msg: Int64):
      self.current_u1 = msg.data
   
   def update_brake_input(self, msg: Int64):
      self.current_u2 = msg.data

   def update_steering_input(self, msg: Int64):
      self.current_u3 = msg.data



   def update_data(self, msg: TwistWithCovarianceStamped):
      self.time_now_ = time.time() - self.time_start_

      self.velocity_reading_ = msg.twist.twist.linear.x
      self.angular_reading_ = msg.twist.twist.angular.z

      self.time_stamp_.append(self.time_now_)
      self.velocity_list_.append(self.velocity_reading_)
      self.angle_list_.append(self.angular_reading_)
      self.u1_list_.append(self.current_u1)
      self.u2_list_.append(self.current_u2)
      self.u3_list_.append(self.current_u3)

  


def save_data_to_cvs(time, u1, u2, u3, linVel, angVel): 
   data = {'time': time, 'u1':u1, 'u2':u2, 'u3':u3, 'lin_vel':linVel, 'angVel':angVel}
   data_frame = pd.DataFrame(data)
   directory = '~/data_test_ntnu/python_data/'
   timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
   filename = f'data_export_{timestamp}.csv'
   file_path = directory + filename

   data_frame.to_csv(file_path)

def main(args=None):
   rclpy.init(args=args)
   node = Subscriber()
   try: 
      rclpy.spin(node)
   except KeyboardInterrupt:
      print("caught the interrupt, shutting down:) ")
      save_data_to_cvs(node.time_stamp_, node.u1_list_, node.u2_list_,node.u3_list_, node.velocity_list_, node.angle_list_)

   finally: 
      rclpy.shutdown()


if __name__ == '__main__':
   main()



