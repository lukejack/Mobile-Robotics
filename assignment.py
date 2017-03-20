
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

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
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)        
    self.scan = rospy.Subscriber('/turtlebot/scan', LaserScan, self.scan_callback, queue_size=5)
    self.bumper = rospy.Subscriber('/turtlebot/turtlebot/events/bumper', BumperEvent, self.bumper_callback, queue_size=5)
    #Sensor data
    self.scan_data = None
    self.max_depth = 0
    
    #Modes: 0 - spin_search, 1 - move_found, 2 - move_search, 3 - bumper_triggered, 4 - Cylinder around obstruction mode
    self.mode = 1
    self.timer = 0
    self.obstruction_timer = 0
    self.move_time_total = 0
    self.move_time_start = 0
    self.searching = False
    
  def main(self, image):
          #Modes 3 and 4 ovveride cylinder presence in view
          #Cylinder in sight obstruction adjustment
          if (self.mode == 4):
              #Initialisation block
              #Twist on the spot
              self.move_time_start = self.time.time()
              self.twist.angular.z = self.search_angular * 1.5
              self.twist.linear.x = 0
                  
              self.cmd_vel_pub.publish(self.twist)

              #Move down first seen non-obstructed path
              if (self.obstruction != True):
                  if (self.move_time_start == 0):
                      self.move_time_start = self.time.time()
                  self.move_time_total = 2
                  self.move_new_search()
          else:
              #Bumper trigger
              if (self.mode == 3):
                  #After moving for 1 second, move down deep path 
                  if (self.time.time() - self.timer > 1):
                      self.timer = 0
                      self.max_depth = 0
                      self.mode = 4
                  else:
                      #Begin moving backwards and twisting
                      self.twist.linear.x = -self.forward_velocity
                      self.twist.angular.z = self.search_angular
                      self.cmd_vel_pub.publish(self.twist)
              else:
                  #Get moments by colour slicing
                  coords = []
                  for key, value in self.colours.iteritems():
                      if (value['found'] == False):
                          mask = cv2.inRange(image, value['min'], value['max'])
                          coords.append([key, cv2.moments(mask), mask])
                  
                  
                  #Check for each colour cylinder
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
                              #Obstruction avoidance
                              if (self.obstruction()):
                                  self.timer = 0
                                  self.max_depth = 0
                                  self.mode = 4
                                  break
                              
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
                                  if (self.obstruction() or ((self.move_time_total != 0) & (self.move_time_total < (self.time.time() - self.move_time_start) ))):
                                      #Start searching again
                                      self.move_time_total = 0
                                      self.move_time_start = 0
                                      self.twist.linear.x = 0
                                      self.mode = 1
                                      self.max_depth = 0
                                      self.move_time_total = 0
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
                                     #Code to check if current deepest path is deep enough / if its max depth
                                     if (self.scan_data[320] > (self.max_depth - 0.4)):
                                          #If obstructed, avoid obstruction
                                          if (self.obstruction()):
                                              self.max_depth = 0
                                              self.timer = 0
                                              self.mode = 4
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

  
  def move_new_search(self):
    self.twist.linear.x = self.forward_velocity
    self.twist.angular.z = 0
    #If has an obstruction
    if (self.obstruction() or ((self.move_time_total != 0) & (self.move_time_total < (self.time.time() - self.move_time_start) ))):
        #Start searching again
        self.move_time_total = 0
        self.move_time_start = 0
        self.twist.linear.x = 0
        self.mode = 1
        self.max_depth = 0
        self.move_time_total = 0
        self.searching = False
    else:
        #Otherwise keep moving
        self.cmd_vel_pub.publish(self.twist)
          
  #Check if there is an obstruction through the laser scanner
  def obstruction(self):
      for p in range(60, 419):
         if (self.scan_data[p] < 1):
             return True
      return False
      
  def scan_callback(self, data):
      self.scan_data = data.ranges
      if ((self.mode == 0) & (data.ranges[320] > self.max_depth)):
          self.max_depth = data.ranges[320]
          
  def bumper_callback(self, data):
      if (data.state == 1 & (self.mode != 3)):
          self.timer = self.time.time()
          self.mode = 3
      
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    self.main(image)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
