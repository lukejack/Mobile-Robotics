
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
    self.forward_velocity = 1
    self.obs_scan_start = 100
    self.obs_scan_end = 390
    
    #Colour slicing
    self.colours = {'red': {'found': False, 'min': numpy.array((0, 0, 1)), 'max': numpy.array((0, 0, 255))},
                    'blue': {'found': False, 'min': numpy.array((100, 0, 0)), 'max': numpy.array((255, 5, 5))},
                    'yellow': {'found': False, 'min': numpy.array((0, 100, 100)), 'max': numpy.array((0, 255, 255))},
                    'green': {'found': False, 'min': numpy.array((0, 10, 0)), 'max': numpy.array((0, 255, 0))}}
        
    self.red_found = False
    self.blue_found = False
    self.yellow_found = False
    self.green_found = False
    
    #ROS Topics
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback, queue_size=10)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=10)        
    self.scan = rospy.Subscriber('/turtlebot/scan', LaserScan, self.scan_callback, queue_size=10)
    
    #Sensor data
    self.scan_data = None
    self.max_depth = 0
    
    #Modes: 0 - spin_search, 1 - move_found, 2 - move_search, 3 - avoid obstruction
    self.mode = 1
    self.mode_3_direction = None
    self.timer = 0
    self.obstruction_timer = 0
    
  def main(self, image):
      
      #Obstruction avoidance mode
      if (self.mode == 3):
          #Initialise timer
          if (self.obstruction_timer == 0):
              self.obstruction_timer = self.time.time()
          #Timed movements to adjust position
          if ((self.time.time() - self.obstruction_timer) > 0.5):
              if ((self.time.time() - self.obstruction_timer) > 1):
                  if ((self.time.time() - self.obstruction_timer) > 1.5):
                      if ((self.time.time() - self.obstruction_timer) > 2):
                          print 'Correction: Finished'
                          self.twist.linear.x = 0
                          self.twist.angular.x = 0
                          self.mode = 1
                          self.obstruction_timer = 0
                      else:
                          print 'Correction: Turning back'
                          self.twist.linear.x = 0
                          if (self.mode_3_direction == 1):
                              self.twist.angular.x = -self.search_angular
                          if (self.mode_3_direction == 2):
                              self.twist.angular.x = self.search_angular
                  else:
                      print 'Correction: Moving forwards'
                      self.twist.angular.x = 0
                      self.twist.linear.x = self.forward_velocity
              else:
                  self.twist.linear.x = 0
                  if (self.mode_3_direction == 1):
                      self.twist.angular.x = self.search_angular
                      print "Turning to the right"
                  if (self.mode_3_direction == 2):
                      self.twist.angular.x = -self.search_angular
                      print "Turning to the left"
          else:
              print 'Correction: Moving backwards'
              self.twist.linear.x = -self.forward_velocity
          self.cmd_vel_pub.publish(self.twist)
          
      #No obstruction avoidance
      else:
          #Get moments by colour slicing
          coords = []
          for key, value in self.colours.iteritems():
              if (value['found'] == False):
                  mask = cv2.inRange(image, value['min'], value['max'])
                  coords.append([key, cv2.moments(mask), mask])
          
          #Check for each colour post
          for colour in coords:
              #If object seen
              if colour[1]['m00'] > 0:
                  #Reset depth watcher
                  self.max_depth = 0
                  self.mode = 1
                  
                  #Center point of cylinder
                  cx = int(colour[1]['m10']/colour[1]['m00'])
                              
                  #If close to the colour
                  if (colour[1]['m00'] > 13000000):
                      print 'Found ', colour[0], '!'
                      self.colours[colour[0]]['found'] = True
                  else:
                      obs = self.obstruction()
                      if obs:
                          self.mode_3_direction = obs
                          self.mode = 3
                         
                          
                      
                  #Move to the lef / right
                  self.twist.angular.z =  -float(cx - self.image_width / 2) / 100 
                  #Move forwards
                  self.twist.linear.x = self.forward_velocity
                  self.cmd_vel_pub.publish(self.twist)
                  break
              
              #This colour not visible
              else:
                  #If there are none of any colour
                  if (colour[0] == coords[-1][0]):
                      #If moving to a new search place
                      if (self.mode == 2):
                          
                          self.twist.linear.x = self.forward_velocity
                          self.twist.angular.z = 0
                          #If has an obstruction
                          obs = self.obstruction()
                          if obs:
                              #Start searching again
                              self.twist.linear.x = 0
                              self.mode = 1
                              self.max_depth = 0
                              break
                          else:
                              #Otherwise keep moving
                              self.cmd_vel_pub.publish(self.twist)
                      #Start finding max depth in circle
                      else:
                          if (self.mode != 0):
                              #Initialise timer
                              self.max_depth = 0
                              self.mode = 0
                              self.timer = self.time.time()
                          #If a full revolution has occured
                          if ((self.mode == 0) & ((self.time.time() - self.timer) > 1.6 * numpy.pi + 1)):
                             #If looking at deepest path
                             if (self.scan_data[320] > (self.max_depth - 0.4)):
                                  #If obstructed, avoid obstruction
                                  if (self.obstruction()):
                                      self.max_depth = 0
                                      self.mode = 3
                                  #Otherwise move in that direction with mode 2
                                  else:
                                      self.max_depth = 0
                                      self.mode = 2
                          #Turn on the spot
                          self.twist.linear.x = 0
                          self.twist.angular.z = self.search_angular
                          self.cmd_vel_pub.publish(self.twist)

      cv2.imshow("window", image)
      cv2.waitKey(3) 
  
  
  def scan_callback(self, data):
      self.scan_data = data.ranges
      if ((self.mode == 0) & (data.ranges[320] > self.max_depth) ):
          self.max_depth = data.ranges[320]
          
  def obstruction(self):
      obs = False
      
      for p in range(self.obs_scan_start, self.obs_scan_end):
          if (self.scan_data[p] < 1):
              obs = True
              break;
      
      if obs:
          obs_left_average = 0
          obs_right_average = 0
          scan_range = (self.obs_scan_end - self.obs_scan_start)
          for p in range(self.obs_scan_start, 240):
              if math.isnan(self.scan_data[p]) or (self.scan_data[p] > 1) :
                  scan_range -= 1
                  continue
              else:
                  obs_right_average += self.scan_data[p]
          obs_right_average /= scan_range
          
          
          scan_range = (self.obs_scan_end - self.obs_scan_start)
          for p in range(241, self.obs_scan_end):
              if math.isnan(self.scan_data[p]) or (self.scan_data[p] > 1) :
                  scan_range -= 1
                  continue
              else:
                  obs_left_average += self.scan_data[p]
                  
          obs_left_average /= scan_range
          
          #print "Obs right average: ", obs_right_average
          #print "Obs left average: ", obs_left_average
          
          obs_diff = obs_right_average - obs_left_average
          
          if (obs_diff > 0.1):
              print "Right obstruction"
              return 2
          if (obs_diff < -0.1):
              print "Left obstruction"
              return 1
          if ((obs_diff < 0.2) & (obs_diff > -0.2)):
              print "Front obstruction"
              return 3
      else:
          return False
      
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    self.main(image)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL