
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
import struct
import ctypes
import sys
import time
import math
from roslib import message
from sensor_msgs.msg import Image, PointCloud2 as Pc2
from sensor_msgs import point_cloud2 as Pc_2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    
    #Libraries
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.twist = Twist()
    self.time = time
    
    #Constants
    self.image_width = 640
    self.image_height = 480
    self.seen_angular = 0.3
    self.search_angular = 0.8
    self.forward_velocity = 0.7
    
    #Colour slicing
    self.colours = {'red': {'found': False, 'min': numpy.array((0, 0, 1)), 'max': numpy.array((0, 0, 255))},
                    'blue': {'found': False, 'min': numpy.array((1, 0, 0)), 'max': numpy.array((255, 0, 0))},
                    'yellow': {'found': False, 'min': numpy.array((100, 100, 0)), 'max': numpy.array((255, 255, 0))},
                    'green': {'found': False, 'min': numpy.array((0, 1, 0)), 'max': numpy.array((0, 255, 0))}}
        
    self.red_found = False
    self.blue_found = False
    self.yellow_found = False
    self.green_found = False
    
    #ROS Topics
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)        
    self.scan = rospy.Subscriber('/turtlebot/scan', LaserScan, self.scan_callback, queue_size=5)
    
    #Sensor data
    self.scan_data = None
    self.max_depth = 0
    
    #Modes: 0 - spin_search, 1 - move_found, 2 - move_search
    self.mode = 1
    self.timer = 0
    
  def main(self, bgr):
      #Get moments by colour slicing
      coords = []
      for key, value in self.colours.iteritems():
          if (value['found'] == False):
              coords.append([key, cv2.moments(cv2.inRange(bgr, value['min'], value['max']))])
      
      #Check for each colour post
      for colour in coords:
          #If object seen
          if colour[1]['m00'] > 0:
              self.max_depth = 0
              self.mode = 1
              cx = int(colour[1]['m10']/colour[1]['m00'])
              if (self.scan_data[320] < 0.8):
                  print 'Found ', colour[0], '!'
                  self.colours[colour[0]]['found'] = True
              if (cx < self.image_width/2):
                  self.twist.angular.z = self.seen_angular
              else:
                  self.twist.angular.z = -self.seen_angular
              self.twist.linear.x = self.forward_velocity
              self.cmd_vel_pub.publish(self.twist)
              break
          
          #Nothing visible
          else:
              #If there are none of any colour
              if (colour[0] == coords[-1][0]):
                  if (self.mode == 2):
                      self.twist.linear.x = self.forward_velocity
                      self.twist.angular.z = 0
                      if (self.scan_data[320] < 1):
                          self.mode = 0
                      else:
                          self.cmd_vel_pub.publish(self.twist)
                  else:
                      if (self.mode != 0):
                          self.max_depth = 0
                          self.mode = 0
                          self.timer = self.time.time()
                      if ((self.mode == 0) & ((self.time.time() - self.timer) > 1.6 * numpy.pi + 1)):
                          if (self.scan_data[320] > self.max_depth - 0.3):
                              #START TIMER
                              self.mode = 2
                          #go to deepest point
                          
                      self.twist.linear.x = 0
                      self.twist.angular.z = self.search_angular
                      
                      self.cmd_vel_pub.publish(self.twist)

      print "Mode: ", self.mode
      print "Max_depth: ", self.max_depth
      cv2.imshow("window", bgr)
      cv2.waitKey(3) 
  
  
  def scan_callback(self, data):
      self.scan_data = data.ranges
      if ((self.mode == 0) & (data.ranges[320] > self.max_depth)):
          self.max_depth = data.ranges[320]
      
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    self.main(image)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL