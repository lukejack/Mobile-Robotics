
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
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
                                       
    self.scan = rospy.Subscriber('/turtlebot/scan', LaserScan, self.scan_callback)
                                      
    self.scan_data = None
    self.twist = Twist()
    self.twist_time = 0
    self.forward_velocity = 1
    self.opening_find = False
    self.opening_time = 0
    
    
  def scan_callback(self, data):
      self.scan_data = data.ranges
      #if (data.ranges[320] < 0.)

  def spin_find(self, bgr):
      #print time.time() - self.twist_time
      #Get image dimensions
      height, width, depth = bgr.shape
      
      hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
      
      #Find red objects
      red_mask = cv2.inRange(bgr, numpy.array((0, 0, 1)), numpy.array((0, 0, 255)))
      blue_mask = cv2.inRange(bgr, numpy.array((1, 0, 0)), numpy.array((255, 0, 0)))
      yellow_mask = cv2.inRange(bgr, numpy.array((100, 100, 0)), numpy.array((255, 255, 0)))
      green_mask = cv2.inRange(bgr, numpy.array((0, 1, 0)), numpy.array((0, 255, 0)))
      
      #If has moved through the opening for 2 seconds
      if ((self.opening_time != 0) & self.opening_find & ((time.time() - self.opening_time) > 2)):
          print "spinning again"
          self.opening_time = 0
          self.opening_find = False
          
      #Clear path found
      if (self.opening_find):
          print "path found"
          scan_temp = self.scan_data
          average_depth = 0
          for i in range(310, 330):
              if (math.isnan(scan_temp[i])):
                  continue
              average_depth = average_depth + scan_temp[i]
         
          average_depth = average_depth // 20
          if (average_depth > 3):
              self.opening_time = time.time()
              self.twist.linear.x = self.forward_velocity
              self.twist.angular.z = 0
          
      else:
          #Get center of red objects
          moments = cv2.moments(red_mask)
          angular_vel = 1
          total_spin = 0
          
          #If an object has been found
          if moments['m00'] > 0:
              self.twist_start = 0
              total_spin = 0
              angle = 0
              cx = int(moments['m10']/moments['m00'])
              cy = int(moments['m01']/moments['m00'])  
              if (cx < width/2):
                  angle = 0.3
              else:
                  angle = -0.3
              self.twist.linear.x = self.forward_velocity
              self.twist.angular.z = angle
          #Nothing visible
          else:
              if (self.twist_time == 0):
                  self.twist_time = time.time()
              if ((time.time() - self.twist_time) > numpy.pi + 0.3):
                  #A full turn has revealed no cylinders
                  self.twist_time = 0
                  self.opening_find = True
                  print 'No cylinders, finding openings'
                  
              #Start twisting on the spot
              self.twist.linear.x = 0
              self.twist.angular.z = angular_vel
      
      self.cmd_vel_pub.publish(self.twist)
      cv2.imshow("window", bgr)
      cv2.waitKey(3)
      
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    (rows, cols, channels) = image.shape
    self.spin_find(image)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL