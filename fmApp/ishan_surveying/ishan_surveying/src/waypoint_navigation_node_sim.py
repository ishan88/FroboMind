#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
2013-06-06 KJ First version
2013-10-01 KJ Fixed a bug causing the velocity to always be the maximum velocity.
              Added launch file paremeters for the waypoint navigation.
2013-11-13 KJ Added new launch file parameters for waypoint defaults
              Added support for implement command and wait after wpt arriva.
2013-12-03 KJ Added ramp up which works like the previous ramp down
"""
"""
03-12-2014 IG Added facility for checking whether the next waypoint is the same as current waypoint
"""
# imports
import rospy
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy # data obtained from a joystick
# from msg import FloatStamped, FloatArrayStamped, waypoint_navigation_status
from math import pi, atan2
from waypoint_list import waypoint_list
#from ishan_waypoint_navigation import waypoint_navigation
from kjeld_waypoint_navigation import waypoint_navigation

'''
This class is to be used for testing the surveyor robot code for simulation purposes only.
It initializes a node which controls the waypoint navigation of the surveyor robot. This node 
loads a set of waypoints from the waypoints file. It also checks whether two consecutive set of
waypoints are similar.
'''
class WptNavNode():
    def __init__(self):
        self.floatStamped = 0.0
        self.floatArrayStamped = []
        
        # Sets the values of all the different parameters of the robot to null values initially
        self.waypoint_navigation_status = {'stamp':'', 'state':'', 'mode':'','b_id':'','b_easting':'',
                                      'b_northing':'', 'a_easting':'', 'a_northing':'', 'easting':'',
                                      'northing':'', 'distance_to_b':'', 'bearing_to_b':'', 
                                      'heading_err':'', 'distance_to_ab_line':'', 'target_easting':'',
                                      'target_northing':'', 'target_distance':'', 'target_bearing':'',
                                      'target_heading_err':'', 'linear_speed':'', 'angular_speed':''}
        
     
        self.sim_wait=6
        self.update_rate = 20 # set update frequency [Hz]
        self.debug_count=0
        self.IMPLEMENT_INVALID = -10000000.0
        self.STATE_IDLE = 0
        self.STATE_NAVIGATE = 1
        self.STATE_WAIT = 2
        self.state = self.STATE_IDLE # sets the initial state of the robot to IDLE
        self.state_prev = self.state
        self.automode_warn = False
        self.wait_after_arrival = 0.0 # wait after the robot reaches a wpt
        self.wait_timeout = 0.0
        self.status = 0
        self.wpt = False
        self.prev_wpt = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.pos = False
        self.bearing = False

        rospy.loginfo(rospy.get_name() + ": Start")
        self.quaternion = np.empty((4, ), dtype=np.float64)
        self.wii_a = False
        self.wii_a_changed = False
        self.wii_plus = False
        self.wii_plus_changed = False
        self.wii_minus = False
        self.wii_minus_changed = False
        self.wii_up = False
        self.wii_up_changed = False
        self.wii_down = False
        self.wii_down_changed = False
        self.wii_left = False
        self.wii_left_changed = False
        self.wii_right = False
        self.wii_right_changed = False
        self.wii_home = False
        self.wii_home_changed = False

        self.debug = rospy.get_param("~print_debug_information", 'true') 
        if self.debug:
            rospy.loginfo(rospy.get_name() + ": Debug enabled")
            
        self.status_publish_interval = rospy.get_param("~status_publish_interval", 0) 
        self.pid_publish_interval = rospy.get_param("~pid_publish_interval", 0) 

        # get topic names
        self.automode_topic = rospy.get_param("~automode_sub",'/fmDecision/automode') # Topic for making the robot move in autonomous mode
        self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose') # Topic on which the pose of the robot is published 
        self.joy_topic = rospy.get_param("~joy_sub",'/fmLib/joy')
#         self.joy_topic = rospy.get_param("~keyboard_sub", "/fmHMI/keyboard")
        self.cmdvel_topic = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel') # Topic on which the velocity of the robot is published
        self.implement_topic = rospy.get_param("~implement_pub",'/fmCommand/implement') # Topic on which the velocity of the implement is published if there is any implement attached to the robot
        self.wptnav_status_topic = rospy.get_param("~status_pub",'/fmInformation/wptnav_status') # Topic which publishes the status of the robot
        self.pid_topic = rospy.get_param("~pid_pub",'/fmInformation/wptnav_pid') # Topic on which the pid info is published

        self.cmd_vel_pub = rospy.Publisher(self.cmdvel_topic, TwistStamped)
        self.twist = TwistStamped()
#         self.implement_pub = rospy.Publisher(self.implement_topic, FloatStamped)
#         self.implement_pub = rospy.Publisher(self.implement_topic, Float)
#         self.implement = FloatStamped()
        self.implement = self.floatStamped
#         self.wptnav_status_pub = rospy.Publisher(self.wptnav_status_topic, waypoint_navigation_status)
        self.wptnav_status = self.waypoint_navigation_status
        self.status_publish_count = 0
#         self.pid_pub = rospy.Publisher(self.pid_topic, FloatArrayStamped)
#         self.pid_pub = rospy.Publisher(self.pid_topic, np.array(np.float64))   
#         self.pid = FloatArrayStamped()
        self.pid = self.floatArrayStamped
        self.pid_publish_count = 0

        # configure waypoint navigation
        self.w_dist = rospy.get_param("/diff_steer_wheel_distance", 0.2) # [m] distance between the two wheels
        drive_kp = rospy.get_param("~drive_kp", 0.1)# driving constant for proportion term in pid controller.
                                                    # Kp, Ki, Kd Values set here are not the true values as the real values are set from the launch file.
        drive_ki = rospy.get_param("~drive_ki", 0.0)# driving constant for integral term in pid controller
        drive_kd = rospy.get_param("~drive_kd", 0.0)# driving constant for differential term in pid controller
        drive_ff = rospy.get_param("~drive_feed_forward", 0.0)# feed forward velocity. It determines the velocity of the robot in response to some stimuli.
                                                              # like for example when a door is opened a feedforward control would heat the room before
                                                              # it gets too cold
        drive_max_output = rospy.get_param("~drive_max_output", 0.3) 
        turn_kp = rospy.get_param("~turn_kp", 1.0)# turning const for proportion term in pid controller
        turn_ki = rospy.get_param("~turn_ki", 0.0)# turning const for integral term in pid controller
        turn_kd = rospy.get_param("~turn_kd", 0.2)# turning const for differential term in pid controller
        turn_ff = rospy.get_param("~turn_feed_forward", 0.0)# feed forward term for turning
        turn_max_output = rospy.get_param("~turn_max_output", 0.5) 

        max_linear_vel = rospy.get_param("~max_linear_velocity", 0.4)
        max_angular_vel = rospy.get_param("~max_angular_velocity", 0.4)

        self.wpt_def_tolerance = rospy.get_param("~wpt_default_tolerance", 0.5)#default tolerance when the robot reaches the waypoint
        self.wpt_def_drive_vel = rospy.get_param("~wpt_default_drive_velocity", 0.5)# default driive velocity when the robot reaches the wpt
        self.wpt_def_turn_vel = rospy.get_param("~wpt_default_turn_velocity", 0.3)# default turn velocity when the robot reaches the wpt
        self.wpt_def_wait_after_arrival = rospy.get_param("~wpt_default_wait_after_arrival", 0.0)# default wait time after robot reaches the wpt
        self.wpt_def_implement = rospy.get_param("~wpt_default_implement_command", 0.0) # default value for the implement stored in the implement param which is a floatstamped message

        turn_start_at_heading_err = rospy.get_param("~turn_start_at_heading_err", 20.0)
        turn_stop_at_heading_err = rospy.get_param("~turn_stop_at_heading_err", 2.0)
        ramp_drive_vel_at_dist = rospy.get_param("~ramp_drive_velocity_at_distance", 1.0)
        ramp_min_drive_vel = rospy.get_param("~ramp_min_drive_velocity", 0.1)
        ramp_turn_vel_at_angle = rospy.get_param("~ramp_turn_velocity_at_angle", 25.0)
        ramp_min_turn_vel = rospy.get_param("~ramp_min_turn_velocity", 0.05)
        stop_nav_at_dist = rospy.get_param("~stop_navigating_at_distance", 0.1)

        # Initializes a waypoint navigation class which contains the waypoint navigation algorithm 
        self.wptnav = waypoint_navigation(self.update_rate, self.w_dist, drive_kp, drive_ki, drive_kd, drive_ff, drive_max_output, turn_kp, 
                                          turn_ki, turn_kd, turn_ff, turn_max_output, max_linear_vel, max_angular_vel, self.wpt_def_tolerance, 
                                          self.wpt_def_drive_vel, self.wpt_def_turn_vel, turn_start_at_heading_err, turn_stop_at_heading_err, 
                                          ramp_drive_vel_at_dist, ramp_min_drive_vel, ramp_turn_vel_at_angle, ramp_min_turn_vel,
                                          stop_nav_at_dist, self.debug)
         
        self.wptlist = waypoint_list() # list of waypoints are loaded from waypoints_list class
        self.wptlist_loaded = False # status whether the waypoints have been loaded or not. Initially set to false

        # setup subscription topic callbacks
        rospy.Subscriber(self.automode_topic, Bool, self.on_automode_message)
        rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)
        rospy.Subscriber(self.joy_topic, Joy, self.on_joy_message)

        # call updater function
        self.r = rospy.Rate(self.update_rate)# update rate 20Hz
        
        # quaternion angles to represent the robot direction
        self.q1=0
        self.q2=0
        self.q3=0
        self.q4=0
        self.updater()

    '''
    Loads the waypoints from the text file 
    '''
    def load_wpt_list (self):
        self.wptlist.load_from_csv_ne_format ("/home/ishan/ish_frobomind/src/fmApp/ishan_surveying/ishan_surveying/waypoints/waypoints_sim.txt")# loads the waypoints from the text file
        (numwpt, nextwpt) = self.wptlist.status()# numwpt - contains the total no of wpts present in the waypoints file. next wpt - The next wpt that robot should move to. Initially next wpt is 0.
        self.prev_wpt = False # No previous wpt. Initially false
        self.wpt = False  # current wpt also set to false initially
        rospy.loginfo(rospy.get_name() + ": %d waypoints loaded" % numwpt)
    
    
    def update_implement_value (self):
        if self.wpt[self.wptnav.W_IMPLEMENT] != self.IMPLEMENT_INVALID:# checks if the wpt param contains the index assigned to W_Implement which is
                                                                       # and checks the value assigned to it. If it is not equal to implement_invalid, then it assigns the implement variable which is a float stamped message with the value contained at index 9 in the waypoints.txt file
            #self.implement.data = self.wpt[self.wptnav.W_IMPLEMENT]
            self.implement = self.wpt[self.wptnav.W_IMPLEMENT]
        else:
            self.implement = self.wpt_def_implement # If the text file contains an implement invalid data then pass the default value of the implement to the implement variable
            #self.implement.data = self.wpt_def_implement
    
    '''
    Checks whether the next waypoint that the robot needs to move is the same as the current
    waypoint. If it is same then it would skip the next waypoint.
    '''
    def isSameWpt(self):
        if(self.prev_wpt==self.wpt):
            rospy.loginfo("The next waypoint is the same as the current waypoint. Skipping Waypoint")
            return True
        else:
            rospy.loginfo("The next waypoint is different from current waypoint.")
            return False
    
    '''
    Instructs the robot to move to the next waypoint. If the next wpt is the same as the current
    wpt then the robot loads the next wpt from the list.
    '''    
    def goto_next_wpt (self):
        
        self.prev_wpt = self.wpt # sets the previous waypoint to the current waypoint
        while(self.isSameWpt()):
            self.wpt = self.wptlist.get_next()
            
            
        if self.wpt != False:
            self.update_implement_value()
            self.wptnav.navigate(self.wpt, self.prev_wpt)
            rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s (distance %.2fm, bearing %.0f)" % (self.wpt[self.wptnav.W_ID], self.wptnav.dist, self.wptnav.bearing*180.0/pi))
        else:
            rospy.loginfo(rospy.get_name() + ": End of waypoint list reached")
            self.wptnav.stop()
    
    '''
    Instructs the robot to move to the previous waypoint. 
    '''
    def goto_previous_wpt (self):
        (wpt, prev_wpt) = self.wptlist.get_previous()# gets the previous waypoint from the waypoints txt file
        if wpt != False:
            self.wpt = wpt
            self.prev_wpt = prev_wpt
            self.update_implement_value()
            self.wptnav.navigate(self.wpt, self.prev_wpt)
            rospy.loginfo(rospy.get_name() + ": Navigating to waypoint: %s (distance %.2fm, bearing %.0f)" % (self.wpt[self.wptnav.W_ID], self.wptnav.dist, self.wptnav.bearing*180.0/pi))
        else:
            rospy.loginfo(rospy.get_name() + ": This is the first waypoint")

    '''
    A callback method when the user presses the automode button from either a keyboard or a 
    wiimote.
    '''
    def on_automode_message(self, msg):
        
        if msg.data == True: # if autonomous mode requested
            if self.state == self.STATE_IDLE:
                if self.wptnav.pose != False: # if we have a valid pose                
                    self.state = self.STATE_NAVIGATE
                    rospy.loginfo(rospy.get_name() + ": Switching to waypoint navigation")
                    if self.wptlist_loaded == False: # If wpts not already loaded then load the wpts and then make the robot move to the next wpt
                        rospy.loginfo(rospy.get_name() + ": Loading waypoint list")
                        self.load_wpt_list()                
                        self.goto_next_wpt()
                        self.wptlist_loaded = True
                    elif self.wptnav.state == self.wptnav.STATE_STANDBY: # If robot is in wait state then make the robot resume the navigation
                        self.wptnav.resume()
                        rospy.loginfo(rospy.get_name() + ": Resuming waypoint navigation")

                else: # no valid pose yet
                    if self.automode_warn == False:
                        self.automode_warn = True
                        rospy.logwarn(rospy.get_name() + ": Absolute pose is required for autonomous navigation")
        else: # if manual mode requested
            if self.state != self.STATE_IDLE:
                self.state = self.STATE_IDLE                    
                self.wptnav.standby() 
                rospy.loginfo(rospy.get_name() + ": Switching to manual control")            
   
    '''
    A callback method which is called when a valid pose is received from the robot
    ''' 
    def on_pose_message(self, msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        self.wptnav.state_update (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x)

    
    def on_joy_message(self, msg):
        if int(msg.buttons[2]) != self.wii_a:
            self.wii_a =  int(msg.buttons[2])
            self.wii_a_changed = True
        if self.state != self.STATE_IDLE and int(msg.buttons[8]) != self.wii_up:
            self.wii_up =  int(msg.buttons[8])
            self.wii_up_changed = True
        if self.state != self.STATE_IDLE and int(msg.buttons[9]) != self.wii_down:
            self.wii_down =  int(msg.buttons[9])
            self.wii_down_changed = True
        if int(msg.buttons[10]) != self.wii_home:
            self.wii_home =  int(msg.buttons[10])
            self.wii_home_changed = True
    
    '''
    A method which publishes the linear and angular velocity of the robot on topics
    '''
    def publish_cmd_vel_message(self):
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = self.linear_vel
        self.twist.twist.angular.z = self.angular_vel        
        self.cmd_vel_pub.publish (self.twist)

    def publish_implement_message(self):
        print("")
#         self.implement.header.stamp = rospy.Time.now()
        self.implement = rospy.Time.now()
#         self.implement_pub.publish (self.implement)

    '''
    Sets the status of the different parameters of the robot in a data structure
    '''
    def publish_status_message(self):
#         self.wptnav_status.header.stamp = rospy.Time.now()
        self.wptnav_status['stamp'] = rospy.Time.now()
#         self.wptnav_status.state = self.state
        self.wptnav_status['state'] = self.state
        if self.wptnav.pose != False:
#             self.wptnav_status.easting = self.wptnav.pose[0]
            self.wptnav_status['easting'] = self.wptnav.pose[0]
#             self.wptnav_status.northing = self.wptnav.pose[1]
            self.wptnav_status['northing'] = self.wptnav.pose[1]

        if self.state == self.STATE_NAVIGATE and self.wptnav.b != False:
            if  self.wptnav.state == self.wptnav.STATE_STOP or self.wptnav.state == self.wptnav.STATE_STANDBY:
#                 self.wptnav_status.mode = 0
                self.wptnav_status['mode'] = 0
            elif self.wptnav.state == self.wptnav.STATE_DRIVE:
#                 self.wptnav_status.mode = 1
                 self.wptnav_status['mode']=1
            
            elif self.wptnav.state == self.wptnav.STATE_TURN:
                self.wptnav_status['mode']=2
#                 self.wptnav_status.mode = 2
#             self.wptnav_status.b_easting = self.wptnav.b[self.wptnav.W_E]
#             self.wptnav_status.b_northing = self.wptnav.b[self.wptnav.W_N]
#             self.wptnav_status.a_easting = self.wptnav.a[self.wptnav.W_E]
#             self.wptnav_status.a_northing = self.wptnav.a[self.wptnav.W_N]
#             self.wptnav_status.distance_to_b = self.wptnav.dist
#             self.wptnav_status.bearing_to_b = self.wptnav.bearing
#             self.wptnav_status.heading_err = self.wptnav.heading_err
#             self.wptnav_status.distance_to_ab_line = self.wptnav.ab_dist_to_pose
            
            self.wptnav_status['b_easting'] = self.wptnav.b[self.wptnav.W_E]
            self.wptnav_status['b_northing'] = self.wptnav.b[self.wptnav.W_N]
            self.wptnav_status['a_easting'] = self.wptnav.a[self.wptnav.W_E]
            self.wptnav_status['a_northing'] = self.wptnav.a[self.wptnav.W_N]
            self.wptnav_status['distance_to_b'] = self.wptnav.dist
            self.wptnav_status['bearing_to_b'] = self.wptnav.bearing
            self.wptnav_status['heading_err'] = self.wptnav.heading_err
            self.wptnav_status['distance_to_ab_line'] = self.wptnav.ab_dist_to_pose
            if self.wptnav.target != False:
#                 self.wptnav_status.target_easting = self.wptnav.target[0]
#                 self.wptnav_status.target_northing = self.wptnav.target[1]
#                 self.wptnav_status.target_distance = self.wptnav.target_dist
#                 self.wptnav_status.target_bearing = self.wptnav.target_bearing
#                 self.wptnav_status.target_heading_err = self.wptnav.target_heading_err
                
                self.wptnav_status['target_easting'] = self.wptnav.target[0]
                self.wptnav_status['target_northing'] = self.wptnav.target[1]
                self.wptnav_status['target_distance'] = self.wptnav.target_dist
                self.wptnav_status['target_bearing'] = self.wptnav.target_bearing
                self.wptnav_status['target_heading_err'] = self.wptnav.target_heading_err
            else:    
#                 self.wptnav_status.target_easting = 0.0
#                 self.wptnav_status.target_northing = 0.0
#                 self.wptnav_status.target_distance = 0.0
#                 self.wptnav_status.target_bearing = 0.0
#                 self.wptnav_status.target_heading_err = 0.0
                
                self.wptnav_status['target_easting'] = 0.0
                self.wptnav_status['target_northing'] = 0.0
                self.wptnav_status['target_distance'] = 0.0
                self.wptnav_status['target_bearing'] = 0.0
                self.wptnav_status['target_heading_err'] = 0.0
#             self.wptnav_status.linear_speed = self.wptnav.linear_vel
#             self.wptnav_status.angular_speed = self.wptnav.angular_vel
            self.wptnav_status['linear_speed'] = self.wptnav.linear_vel
            self.wptnav_status['angular_speed'] = self.wptnav.angular_vel
        else:
#             self.wptnav_status.mode = -1
            self.wptnav_status['mode'] = -1            
#         self.wptnav_status_pub.publish (self.wptnav_status)

    '''
    Sets the status of the PID in a data structure
    '''
    def publish_pid_message(self):
        if self.state == self.STATE_NAVIGATE:
            self.pid.append(rospy.Time.now())
            self.pid.append(self.wptnav.pid_status)
#             self.pid_pub.publish (self.pid)

    '''
    A method which continously checks the status of the wiimote and checks whether the robot
    has reached the destination.
    '''
    def updater(self):        
        while not rospy.is_shutdown():                 
            if self.wii_a == True and self.wii_a_changed == True:
                self.wii_a_changed = False
                rospy.loginfo(rospy.get_name() + ': Current position: %.3f %.3f' % (self.wptnav.pose[0], self.wptnav.pose[1]))# prints the inital pose of the robot 
            if self.wii_home == True and self.wii_home_changed == True:      # waypoint list has been reloaded and been signalled from the wiimote                                                                
                self.wii_home_changed = False
                rospy.loginfo(rospy.get_name() + ": User reloaded waypoint list")
                self.load_wpt_list()                
                self.goto_next_wpt()
            if self.wii_up == True and self.wii_up_changed == True:
                self.wii_up_changed = False
                rospy.loginfo(rospy.get_name() + ": User skipped waypoint")
                self.goto_next_wpt()
            if self.wii_down == True and self.wii_down_changed == True:
                self.wii_down_changed = False
                rospy.loginfo(rospy.get_name() + ": User selected previous waypoint")
                self.goto_previous_wpt()

            if self.state == self.STATE_NAVIGATE:
                (self.status, self.linear_vel, self.angular_vel) = self.wptnav.update(rospy.get_time())
                self.debug_count+=1
   
                if self.status == self.wptnav.UPDATE_ARRIVAL:
                    rospy.loginfo(rospy.get_name() + ": Arrived at waypoint: "+str(self.wpt[self.wptnav.W_ID])+" Error = "+str((self.wptnav.dist)*100)+" cm"+" Pose = "+str(self.wptnav.pose))
                    rospy.loginfo(rospy.get_name() + ": Waiting %.1f seconds" % (self.sim_wait))
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    self.publish_cmd_vel_message()
                    rospy.sleep(6)

                     #activate wait mode
                    if self.wpt[self.wptnav.W_WAIT] >= 0.0:
                        self.wait_after_arrival = self.wpt[self.wptnav.W_WAIT]
                    else:
                        self.wait_after_arrival = self.wpt_def_wait_after_arrival

                    if self.state == self.STATE_WAIT:
                        self.wait_timeout = rospy.get_time() + self.wait_after_arrival
                       
                        
                    else:
                        self.state = self.STATE_NAVIGATE 
                        rospy.loginfo(" Inside updater. Moving to next wpt")        
                        self.goto_next_wpt()

                else:
                    self.publish_cmd_vel_message()
                    self.publish_implement_message()

            elif self.state == self.STATE_WAIT:
                rospy.loginfo("Inside updater. Robot is in wait state.")
                if rospy.get_time() > self.wait_timeout:
                    self.state = self.STATE_NAVIGATE # So, when a manual mode is requested the control comes here when the wait timeout is reached. Here it switches the state to the Navigate State.        
                    rospy.loginfo("Inside updater. Get next wpt from wait state")
                    self.goto_next_wpt()
                else:                
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    self.publish_cmd_vel_message()
                    self.publish_implement_message()

            # publish status
            if self.status_publish_interval != 0:
                self.status_publish_count += 1
                if self.status_publish_count % self.status_publish_interval == 0:
                    self.publish_status_message()

            # publish pid
            if self.pid_publish_interval != 0:
                self.pid_publish_count += 1
                if self.pid_publish_count % self.pid_publish_interval == 0:
                    self.publish_pid_message()
            self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('wptnav_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = WptNavNode()
    except rospy.ROSInterruptException:
        pass

