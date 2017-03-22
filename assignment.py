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
    self.search_angular = 0.8
    self.forward_velocity = 1
    self.moments_large = 13000000 #Minimum moment size for close proximity
    self.time_before_panic = 45 #Search duration without finding cylider before behaviour change
    self.panic_duration = 10 #Time before behaviour reverts after panic
    
    #Colour slicing
    self.colours = {'red': {'found': False, 'min': numpy.array((0, 0, 1)), 'max': numpy.array((0, 0, 255))},
                    'blue': {'found': False, 'min': numpy.array((100, 0, 0)), 'max': numpy.array((255, 5, 5))},
                    'yellow': {'found': False, 'min': numpy.array((0, 100, 100)), 'max': numpy.array((0, 255, 255))},
                    'green': {'found': False, 'min': numpy.array((0, 10, 0)), 'max': numpy.array((0, 255, 0))}}
    
    #ROS Topics
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)        
    self.scan = rospy.Subscriber('/turtlebot/scan', LaserScan, self.scan_callback, queue_size=5)
    self.bumper = rospy.Subscriber('/turtlebot/turtlebot/events/bumper', BumperEvent, self.bumper_callback, queue_size=5)
   
    #Sensor data
    self.scan_data = None
    
    #Modes: 0 - spin_search, 1 - move_found, 2 - move_search, 3 - bumper_triggered, 4 - Cylinder around obstruction mode, 5 - Panic mode  
    self.mode = 1 #Default: 1
    self.bump = 0
    self.panic_timer = 0
    self.timer = 0
    self.found_timer = self.time.time()
    self.first_path = False
    self.reset_data()
        
    
  def main(self, image):
          #Modes 3, 4, and 5 ovveride cylinder presence in view
  
          #If no cylinder has been found for a certain time period, enable panic mode
          if ((self.found_timer != 0) & ((self.time.time() - self.found_timer) > self.time_before_panic)):
              self.found_timer = self.time.time()
              self.mode = 5
          
          #Panic mode prioritises first depth movement for a time period in order to find a new area
          if (self.mode == 5):
              #Begin the counter to check the panic duration
              if (self.panic_timer == 0):
                  self.panic_timer = self.time.time()
              #Set the movement mode to find the first non-obstructed path for the panic duration
              self.first_path = True
              self.move_first(self.panic_duration)
          else:
              #If in move around obstruction mode
              if (self.mode == 4):
                  #Move to the first visible path
                  self.move_first(0)
              else:
                  #Bumper trigger
                  if (self.mode == 3):
                      #After moving backwards for 1 second, move first visible path
                      if (self.time.time() - self.timer > 1):
                          self.reset_data()
                          self.mode = 4
                      else:
                          #Begin moving backwards and twisting
                          self.twist.linear.x = -self.forward_velocity
                          self.twist.angular.z = self.search_angular
                  else:
                      #Get moments by colour slicing
                      sliced = []
                      for key, value in self.colours.iteritems():
                          if (value['found'] == False):
                              mask = cv2.inRange(image, value['min'], value['max'])
                              sliced.append([key, cv2.moments(mask), mask])
                      
                      #Sort colours in order of their size on the screen
                      swapped = True
                      while swapped:
                          swapped = False
                          for i in range(0, len(sliced) - 1):
                              if (i == len(sliced) - 1):
                                  continue
                              else:
                                  if (sliced[i + 1][1]['m00'] > sliced[i][1]['m00']):
                                      temp = sliced[i]
                                      sliced[i] = sliced[i + 1]
                                      sliced[i + 1] = temp
                                      swapped = True
                                      
                      #Tell the user if all cylinders have been found
                      cylinders_left = False
                      for key, value in self.colours.iteritems():
                          if (value['found'] == False):
                              cylinders_left = True
                      if (not cylinders_left):
                          self.time_before_panic = 10000
                          print "All colours found!"

                      
                     
                      #Iterate over each mask checking for cylinder presence
                      for colour in sliced:
                          #If object seen
                          if colour[1]['m00'] > 0:
                              #Reset depth watcher
                              self.max_depth = 0
                              self.mode = 1
                              
                              #Center point of cylinder
                              cx = int(colour[1]['m10']/colour[1]['m00'])
                                          
                              #If close to the colour, mark it as found
                              if (colour[1]['m00'] > self.moments_large):
                                  print 'Found ', colour[0], '!'
                                  self.found_timer = self.time.time()
                                  self.colours[colour[0]]['found'] = True
                              else:
                                  #Avoid obstruction if cylinder is in view but bot is obstructed
                                  if (self.obstruction()):
                                      self.reset_data()
                                      self.first_path = True
                                      self.mode = 4
                                      break
                                  
                              #Twist with a velocity proportional to offset of the cylinder from center
                              self.twist.angular.z =  -float(cx - self.image_width / 2) / 100 
                              #Move forwards
                              self.twist.linear.x = self.forward_velocity
                              break
                          
                          #This colour not visible
                          else:
                              #If there are none of any colour
                              if (colour[0] == sliced[-1][0]):
                                  #If moving to a new search place, check for the stopping condition
                                  if (self.mode == 2):
                                      self.move_new_place(0, 0);
                                      break;
                                  
                                  else:
                                      #If not moving to a new search place, start finding max depth path in full circle
                                      if (self.mode != 0):
                                          #Initialise timer to determine a full rotation
                                          self.max_depth = 0
                                          self.mode = 0
                                          self.timer = self.time.time()
                                      #If a full revolution has occured
                                      if ((self.mode == 0) & ((self.time.time() - self.timer) > 1.6 * numpy.pi + 1)):
                                         #If currently looking at the deepest path
                                         if (self.scan_data[320] > (self.max_depth - 0.4)):
                                              #If obstructed, avoid obstruction
                                              if (self.obstruction()):
                                                  self.reset_data()
                                                  self.first_path = True
                                                  self.mode = 4
                                              #Otherwise move in that direction with mode 2
                                              else:
                                                  self.max_depth = 0
                                                  self.mode = 2
                                      #Turn on the spot to scan for max depth in circle
                                      self.twist.linear.x = 0
                                      self.twist.angular.z = self.search_angular
          #Publish any twist commands and show camera stream
          self.cmd_vel_pub.publish(self.twist);
          cv2.imshow("window", image)
          cv2.waitKey(3) 
  
  def move_first(self, duration):
      #Twist on the spot
      self.twist.angular.z = self.search_angular * 1.5
      self.twist.linear.x = 0

      #Move down unobstructed path
      if ((self.first_path == False) or ((self.first_path == True) & (self.obstruction() != True))):
          #initialise timer
          if (self.timer == 0):
              self.timer = self.time.time()
          #First path added
          if (duration == 0):
              self.first_path = False
              #Move to first visible path for 2 seconds max
              self.move_new_place(2, 0)
          else:
              #Duration determines first visible depth movement for a timer period (panic mode)
              self.move_new_place(2, duration)
  
  #Move to a new place
  def move_new_place(self, move_time, persistent_time):
    self.twist.linear.x = self.forward_velocity
    self.twist.angular.z = 0
    #If has an obstruction, or it has moved for the set move duration
    if (self.obstruction() or ((move_time != 0) & (move_time < (self.time.time() - self.timer) ))):
        #Start searching again
        self.timer = 0
        self.twist.linear.x = 0
        #If persistent mode, persist
        if (persistent_time > 0):
            #If the panic mode has finished
            if ((self.time.time() - self.panic_timer) > persistent_time):
                #Return to normal operation
                self.panic_timer = 0
                #Switch turn direction to alter behaviour
                self.search_angular *= -1
                self.mode = 1
            else:
                #Continue with panic mode
                self.mode = 5
        else:
            #Return to normal operation if not persistent
            self.mode = 1
        #Reset max depth
        self.max_depth = 0
        
    else:
        #Otherwise keep moving
        self.cmd_vel_pub.publish(self.twist)
  
  #Reset max_depth and timer
  def reset_data(self):
      self.max_depth = 0
      self.timer = 0
          
  #Check if there is an obstruction through the laser scanner
  def obstruction(self):
      for p in range(60, 419):
         if (self.scan_data[p] < 0.7):
             return True
      return False
      
  def scan_callback(self, data):
      self.scan_data = data.ranges
      if ((self.mode == 0) & (data.ranges[320] > self.max_depth)):
          self.max_depth = data.ranges[320]
          
  def bumper_callback(self, data):
      #Avoidance mode for objects unseen by the depth scanner
      if (data.state == 1 & (self.mode != 3) & (self.mode != 5)):
          self.timer = self.time.time()
          self.mode = 3
      
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    self.main(image)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
