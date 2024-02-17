#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int64
from geometry_msgs.msg import Twist
#import time
#import math as m
import pandas as pd

class Publisher(Node):

   def __init__(self, input_list1, input_list2, input_list3):
      super().__init__("data_publisher")
      self.u1_ = input_list1
      self.u2_ = input_list2
      self.u3_ = input_list3
      self.u1_counter_ = 0
      self.u2_counter_ = 0
      self.u3_counter_ = 0
      #self.time_start_ = time.time()
      #self.time_now_ = 0 #I might not need this at the moment, this was used for the cosine function
      self.u1_publish_ = self.create_publisher(Int64, '/lw_portena_throttle', 10)
      self.u2_publish_= self.create_publisher(Int64, '/lw_portena_brake', 10)
      self.u3_publish_ = self.create_publisher(Int64, '/lw_portena_steering', 10)
      #self.mock_states = self.create_publisher()
      self.get_logger().info("Publishing inputs")
      self.u1_spin = self.create_timer(0.1, self.publish_u1)
      self.u2_spin = self.create_timer(0.1, self.publish_u2)
      self.u3_spin = self.create_timer(0.1, self.publish_u3)
   
   def publish_u1(self):
      #self.time_now_ = time.time() - self.time_start_ 
      msg = Int64()
      msg.data = self.u1_[self.u1_counter_]
      self.u1_counter_ += 1
      self.get_logger().info("Sending u1")
      self.u1_publish_.publish(msg)
      
   def publish_u2(self):
      #self.time_now_ = time.time() - self.time_start_ 
      msg = Int64()
      msg.data = self.u2_[self.u2_counter_]
      self.u2_counter_ += 1
      self.get_logger().info("Sending u2")
      self.u2_publish_.publish(msg)

   def publish_u3(self):
      #self.time_now_ = time.time() - self.time_start_ 
      msg = Int64()
      msg.data = self.u3_[self.u3_counter_]
      self.u3_counter_ += 1
      self.get_logger().info("Sending u3")
      self.u3_publish_.publish(msg)




def main(args=None):
   input_data = pd.read_csv("~/input_data/inputs/noTurning/inputArrayFiltered/inputArrayFiltered1.csv", delimiter=';')
   input_data['u1'] = input_data['u1'].astype(int)
   input_data['u2'] = input_data['u2'].astype(int)
   input_data['u3'] = input_data['u3'].astype(int)

   list_of_u1 = input_data['u1'].tolist()
   list_of_u2 = input_data['u2'].tolist()
   list_of_u3 = input_data['u3'].tolist()


   rclpy.init(args=args)

   node = Publisher(list_of_u1,list_of_u2,list_of_u3)
   rclpy.spin(node)

   rclpy.shutdown()

if __name__ == '__main__':
   main()
