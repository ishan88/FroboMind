#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
# Copyright (c) 2013-2014, Ishan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or withoutn
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
This waypoint navigation class implements an AB navigation controller

Look at the update function to get an idea of how it works.

Navigation modes:
  STWP: Straight to destination (b) waypoint
  MCTE: Minimize cross track error from origin (a) to destination (b)

2013-06-07 KJ First version
2013-09-22 KJ Added support for initialization of some of the parameters
2013-12-03 KJ Added ramp up which works like the previous ramp down
2014-02-26 KJ Added support for PID feed forward
              Changed target navigation method
"""
"""
04-12-2014 IG Added Log Data Method that would log all the important data in a log file
"""
"""
Waypoint navigation using simple algorithm
"""
# imports
from math import pi, atan2, sqrt, fabs, cos
from differential_kinematics import differential_kinematics
from pid_controller import pid_controller
import rospy
from log_results import LogResults
from log_results_strangeturn import LogResultsStrangeTurn
class waypoint_navigation():
    def __init__(self, update_rate, wheel_dist, drive_kp, drive_ki, drive_kd, drive_ff, drive_max_output, turn_kp, turn_ki, turn_kd, 
                 turn_ff, turn_max_output, max_linear_vel, max_angular_vel, wpt_tol_default, wpt_def_drive_vel, wpt_def_turn_vel, 
                 turn_start_at_heading_err, turn_stop_at_heading_err, ramp_drive_vel_at_dist, ramp_min_drive_vel, ramp_turn_vel_at_angle,
                  ramp_min_turn_vel, stop_nav_at_dist, debug):

        self.param1="Heading Err"
        self.param2 = "Linear Vel"
        self.param3 = "Angular Vel"
        self.param4 = "dist"
        self.param5 = "pose - x"
        self.param6 = "pose - y"
        self.param7 = "pose - z"
        self.param8 = "ab length"
        self.param9 = "Error"
        self.param10 = "Status"
        self.param11 = "ab_dist_to_pose"
        self.param12 = "d"
        self.param13 = "pt[E]"
        self.param14 = "pt[N]"
        self.param15 = "Target[E]"
        self.param16 = "Target[N]"
        self.param17 = "Target_Heading_Error"
        self.arr=False
        self.id=0
        self.log_results_strangeturn = LogResultsStrangeTurn(self.id)
        self.logresults = LogResults(self.id,self.param1,self.param2,self.param3,self.param4,self.param5,
                                     self.param6,self.param7,self.param8,self.param9,self.param10,
                                     self.param11,self.param12,self.param13,self.param14,self.param15,
                                     self.param16,self.param17)#0=SimpleAlgorithm, 1=Kjeld's Algorithm
        self.UPDATE_NONE = 0
        self.UPDATE_ARRIVAL = 1
        self.pi2 = 2.0*pi
        self.deg_to_rad = pi/180.0
        self.rad_to_deg = 180.0/pi
        self.debug_count=0
        self.prev_time_strangeturn = rospy.Time.now().to_sec()

        rospy.loginfo("Ishan Algorithm Running")
        
        # list index for destination (b) and origin (a) waypoints
        self.W_E = 0 
        self.W_N = 1
        self.W_YAW = 2
        self.W_ID = 3
        self.W_MODE = 4
        self.W_TOL = 5
        self.W_LIN_VEL = 6
        self.W_ANG_VEL = 7
        self.W_WAIT = 8
        self.W_IMPLEMENT = 9

        # parameters
        self.update_rate = update_rate # [Hz]
        self.update_interval = (1.0/self.update_rate) # [s]
        self.wheel_dist = wheel_dist
        self.max_linear_vel = max_linear_vel # [m/s]
        self.max_angular_vel = max_angular_vel # [radians/s]
        self.angular_vel_limit = self.max_angular_vel
        self.drive_kp = drive_kp
        self.drive_ki = drive_ki
        self.drive_kd = drive_kd
        self.drive_ff = drive_ff
        self.drive_max_output = drive_max_output
        self.turn_kp = turn_kp
        self.turn_ki = turn_ki
        self.turn_kd = turn_kd
        self.turn_ff = turn_ff
        self.turn_max_output = turn_max_output
        self.wpt_tolerance_default = wpt_tol_default # [m]
        self.wpt_def_drive_vel = wpt_def_drive_vel # [m/s]
        self.wpt_def_turn_vel = wpt_def_turn_vel # [m/s]

        self.turn_start_at_heading_err = turn_start_at_heading_err*self.deg_to_rad # [radians] set to 2pi if not applicable to the robot9
        self.turn_acceptable_heading_err = turn_stop_at_heading_err*self.deg_to_rad # [radians]
        self.ramp_drive_vel_at_dist_default = ramp_drive_vel_at_dist # [m] default velocity for driving in a ramp
        self.ramp_min_drive_vel_default = ramp_min_drive_vel # [m/s]
        self.ramp_turn_vel_at_angle_default = ramp_turn_vel_at_angle*pi/180.0 # [deg] i think converting to radians
        self.ramp_min_turn_vel_default = ramp_min_turn_vel # [rad/s]
        self.stop_nav_at_dist = stop_nav_at_dist

        self.print_interval = self.update_rate/20

        # navigation controller state machine
        self.STATE_STOP = 0 # value when robot is in stop state 
        self.STATE_STANDBY = 1 # value when robot is in standby state
        self.STATE_DRIVE = 2 # value when robot is drive state
        self.STATE_TURN = 3 # value when robot is in turning state

        self.state = self.STATE_STOP # initial state is stop
        self.prev_state = self.STATE_STOP # initial previous state is also stop

        # reset waypoints
        self.start = False 
        self.a = False # starting waypoint for the robot
        self.b = False # Destination waypoint for the robot
        self.pose = False # pose position of the robot. Initially set to false as no pose obtained
        self.vel = 0.0 # velocity of the robot. initially set to 0
        self.target = False # whether robot has reached the target or not. initally false
        self.wpt_tolerance = 0.0 #
        self.wpt_drive_vel = 0.0
        self.wpt_turn_vel = 0.0
        self.wpt_ramp_drive_vel_at_dist = 0.0
        self.wpt_ramp_min_drive_vel = 0.0

        # initialize navigation state vars
        self.dist = 0.0
        self.dist_minimum = 20000000.0# minimum distance at which it is decided that robot has reached the waypooint
        self.bearing = 0.0
        self.heading_err = 0.0 # Current error in the heading of the robot
        self.heading_err_prev = 0.0 # previous error in the heading of the robot
        #self.heading_err_minimum = self.pi2
        self.ab_len = 0.0 # distance betweem the start and the destination waypoints for the robot to navigate
        self.ab_dist_to_pose = 0.0 #  not clear
        self.ab_norm = [0,0] # not clear
        self.target_dist = 0.0 # target distance that robot needs to navigate
        self.target_bearing = 0.0 # target bearing of the robot
        self.target_heading_err = 0.0  # target error position of the heading of the robot
        #self.target_heading_err_prev = 0.0 
        self.turn_bearing_origin = 0.0 # not clear

        # PID drive controller
        self.pid_drive = pid_controller(self.update_interval) # For driving the robot call a pid controller class which initializes the parameters
                                                              #  like kp, ki, kd and which updates the error at a fixed interval of time evry time
        self.pid_drive.set_parameters(self.drive_kp, self.drive_ki, self.drive_kd, self.drive_ff, self.drive_max_output)
        self.pid_status_reset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # pid reset status indicates that all values for pid control are reset to 0
        self.pid_status = self.pid_status_reset # the current status of the pid which is the reset state all 0

        # PID turn controller
        self.pid_turn = pid_controller(self.update_interval) # For turn controller call a pid controller class which initializes the parameters
                                                              #  like kp, ki, kd and which updates the error at a fixed interval of time evry time
        self.pid_turn.set_parameters(self.turn_kp, self.turn_ki, self.turn_kd, self.turn_ff, self.turn_max_output)

        # initialize output
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.debug = debug # initially debug state set to true
        self.prev_time = rospy.Time.now().to_sec()
        
   
    def logData(self,status):
#         rospy.loginfo("Passing Data to Log_Results.py file ")
        self.logresults.file_open()
        self.logresults.log_results(self.heading_err,self.linear_vel,self.angular_vel,self.dist,
                                    self.pose[0],self.pose[1],self.pose[2],self.ab_len," N/A ",
                                    status,"","","","","","","")
        self.logresults.file_close()
    def state_update (self, easting, northing, yaw, vel):
        self.pose = [easting, northing, yaw]
        self.vel = vel
        
     #call to set a new navigation destination waypoint
    def navigate (self, destination, origin):# initially robot only has a destination. Comes here only when automode is requested first time. After that each time it goes to the update method.
#          rospy.loginfo(" IN NAVIGATE() ")
         self.b = destination # where we are going
        # rospy.loginfo("Ishan Algorithm. Inside navigate state. Comes here every time a new waypoint needs to be navigated. Destination is "+str(self.b))
         
         if origin != False:# initially there is no origin of the robot and it is false. So, initally the starting position of the robot is its pose itself.
             self.a = origin
         else:
             self.a = self.pose    
 
         self.start = self.pose # where we started driving from (updated in drive_init())
         #rospy.loginfo("Inside Navigate State.Initially when either a new waypoint is loaded or when the robot starts first time. Variable 'start' is equal to "+str(self.start))
 
         # set velocity and waypoint reached tolerance
         self.wpt_tolerance = float(self.b[self.W_TOL])# assign the wpt tolerance value obtained from the waypoints.txt file
         if self.wpt_tolerance < 0.001:
             self.wpt_tolerance = self.wpt_tolerance_default
 
         # set drive velocity
         self.wpt_drive_vel = float(self.b[self.W_LIN_VEL])# get the linear vel as obtained from the wpt file
         if self.wpt_drive_vel < 0.001:
             self.wpt_drive_vel = self.wpt_def_drive_vel
 
         self.wpt_ramp_min_drive_vel = self.ramp_min_drive_vel_default
         self.wpt_ramp_drive_vel_at_dist = self.ramp_drive_vel_at_dist_default
 
         # set turn velocity limit
         if self.b[self.W_ANG_VEL] > 0.001:
             self.turn_vel_limit = self.b[self.W_ANG_VEL]# set the turning velocity limit equal to the angular velocity obtained from the wpt file
         else:
             self.turn_vel_limit = self.wpt_def_turn_vel
 
         if self.turn_vel_limit > self.max_angular_vel:
                 self.turn_vel_limit = self.max_angular_vel    
 
        
         #rospy.loginfo(" Inside Navigate State. Every time a new waypoint is loaded or robot starts first time then the parameters of the robot like wpt tolerance, linear velocity, angular velocity are initialized. ")
         self.ab_len = sqrt((self.b[self.W_E]-self.a[self.W_E])**2 + (self.b[self.W_N]-self.a[self.W_N])**2) # length of ab line
         self.dist_minimum = 20000000.0
 
         self.update_navigation_state()
         self.drive_init()
    
     # dissected
    # call to stop navigating to the destination
    def stop (self):

        self.b = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.state = self.STATE_STOP
        
    # call to temporaily stop navigating to the destination
    def standby (self):

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.prev_state = self.state
        self.state = self.STATE_STANDBY

    
    # call to resume navigating to the destination
    def resume (self):
        
        self.start = self.pose
        self.state = self.prev_state
        
    # initialize drive towards destination (b) waypoint
    def drive_init(self):
            rospy.loginfo("Driving towards wpt")
            self.start = self.pose #where we started driving from
          #  rospy.loginfo(" Inside drive_init method. start = "+str(self.start))
            self.pid_drive.reset()
           # rospy.loginfo(" Inside drive_init method. PID is reset here.")
            self.state = self.STATE_DRIVE
            #rospy.loginfo(" Robot is in Drive State")
            self.update_navigation_state() # update navigation values immediately
            self.drive() # execute the update immediately
    
    
    # drive towards destination (b) waypoint
    def drive(self):
#         rospy.loginfo(" INSIDE DRIVE() ")
        if fabs(self.target_heading_err) > self.turn_start_at_heading_err and self.dist > self.wpt_tolerance_default:
            self.turn_init()
            #rospy.loginfo(" Turn the robot as heading error is greater than the heading error of the robot at start and robot has not reached near the waypoint.")
        else:
            # determine angular speed
  
            if self.dist > self.stop_nav_at_dist:
                self.angular_vel = self.pid_drive.update (self.target_heading_err) # get controller output
                self.pid_status = self.pid_drive.latest_update_values()
                curr_time = rospy.Time.now().to_sec()
                time_elapsed = curr_time-self.prev_time_strangeturn
                if(time_elapsed>=3):
                    time_elapsed=0
                    self.prev_time_strangeturn=rospy.Time.now().to_sec()
#                     data = " Inside drive method when dist>2.5.Angular velocity here is "+str(self.angular_vel)+" Target Heading Error here is "+str(self.target_heading_err)+" Heading Error is "+str(self.heading_err)
                    self.log_results_strangeturn.file_open()
                    self.log_results_strangeturn.log_results_strangeturn(data)
                    self.log_results_strangeturn.file_close()
#                     rospy.loginfo(" Inside drive method when dist>2.5.Angular velocity here is "+str(self.angular_vel)+" Target Heading Error here is "+str(self.target_heading_err)+" Heading Error is "+str(self.heading_err))
                if(self.dist<=2.5):
                    data1 = " when dist <2.5 "+" Angular velocity here is "+str(self.angular_vel)+" Target Heading Error here is "+str(self.target_heading_err)
                    self.log_results_strangeturn.file_open()
                    self.log_results_strangeturn.log_results_strangeturn(data1) 
                    self.log_results_strangeturn.file_close()
                    rospy.loginfo( "  when dist <2.5 "+"Angular velocity here is "+str(self.angular_vel)+" Target Heading Error here is "+str(self.target_heading_err))
            else:
#                 rospy.loginfo(" INSIDE DRIVE() BLOCK 2 ")
                self.angular_vel = 0.0
  
            # determine linear speed
            self.ramp_dest = 1.0
            self.ramp_start = 1.0
            self.ramp_turn = 1.0
  
            if self.dist < self.wpt_ramp_drive_vel_at_dist: # this is clear now
                self.ramp_dest = self.dist/self.wpt_ramp_drive_vel_at_dist
#                 rospy.loginfo(" 'ramp_dest' = "+str(self.ramp_dest)+" wpt_ramp_drive_vel_at_dist =  "+str(self.wpt_ramp_drive_vel_at_dist))
            if self.dist_start < self.wpt_ramp_drive_vel_at_dist: # close to starting point not clear
                self.ramp_start = self.dist_start/self.wpt_ramp_drive_vel_at_dist # not clear
#                 rospy.loginfo(" 'ramp_start' = "+str(self.ramp_start)+" wpt_ramp_drive_vel_at_dist = "+str(self.wpt_ramp_drive_vel_at_dist))
            wheel_spd  = fabs(self.wheel_dist*self.angular_vel) # [m/s]
            self.ramp_turn = (self.wpt_drive_vel - wheel_spd)/self.wpt_drive_vel
#             rospy.loginfo("  'ramp_turn' = "+str(self.ramp_turn)+" wpt_drive_vel = "+str(self.wpt_drive_vel))
  
            self.ramp = self.ramp_dest
#             rospy.loginfo(" self.ramp = self.ramp_dest, ramp = "+str(self.ramp))

            if self.ramp_start < self.ramp:
                self.ramp = self.ramp_start
#                 rospy.loginfo(" if self.ramp_start < self.ramp:, ramp =  "+str(self.ramp))
            if self.ramp_turn < self.ramp:
                self.ramp = self.ramp_turn
#                 rospy.loginfo(" if self.ramp_turn < self.ramp:, ramp =  "+str(self.ramp))
              
              
            self.linear_vel = self.wpt_drive_vel - (1-self.ramp)*(self.wpt_drive_vel - self.wpt_ramp_min_drive_vel)
              
#             rospy.loginfo(" linear vel = "+str(self.linear_vel)+" dist = "+str(self.dist)+" wpt_drive_vel = "+str(self.wpt_drive_vel)+" wpt_ramp_min_drive_vel = "+str(self.wpt_ramp_min_drive_vel)+"(1-self.ramp)*(self.wpt_drive_vel - self.wpt_ramp_min_drive_vel) = "+str((1-self.ramp)*(self.wpt_drive_vel - self.wpt_ramp_min_drive_vel)))
        
    
      

    # initialize turn about own center (not applicable to all robots)
    def turn_init(self):
        rospy.loginfo(" INSIDE TURN_INIT() ")
        self.pid_turn.reset ()
        self.turn_bearing_origin = self.pose[2]# is the starting bearing angle of the robot
        self.state = self.STATE_TURN
        self.turn() # execute the update immediately

    def limit_value (self, value, maximum):
        if value > maximum:
            value = maximum
        elif value < -maximum:
            value = -maximum
        return value

    # turn about own center (not applicable to all robots)
    def turn(self):
#         rospy.loginfo(" INSIDE TURN() ")
       # rospy.loginfo(" Checks if turn is enough and sets the angular velocity for the turn.")
        angular_dist_to_target = fabs(self.target_heading_err)
        if angular_dist_to_target <= self.turn_acceptable_heading_err:
            self.drive_init()
        else:
# '#             rospy.loginfo(" INSIDE TURN() BLOCK 1 ")'
            pid_error = self.limit_value (self.target_heading_err, pi/4.0)
            self.angular_vel = self.pid_turn.update (pid_error) # get controller output
            self.pid_status = self.pid_turn.latest_update_values()
         #   rospy.loginfo("Calculates pid error ")

            # ramp velocity if close to target
            if angular_dist_to_target < self.ramp_turn_vel_at_angle_default:
#                 rospy.loginfo(" INSIDE TURN() BLOCK 2 ")
                limit = self.turn_vel_limit - (1 - angular_dist_to_target/self.ramp_turn_vel_at_angle_default)*(self.turn_vel_limit - self.ramp_min_turn_vel_default)
            else:
#                 rospy.loginfo(" INSIDE TURN() BLOCK 3")
                limit = self.turn_vel_limit
        
            # ramp velocity if close to origin
            angular_dist_from_origin = fabs(self.angle_diff (self.pose[2], self.turn_bearing_origin))
            if angular_dist_from_origin < self.ramp_turn_vel_at_angle_default:
#                 rospy.loginfo(" INSIDE TURN() BLOCK 4 ")
                origin_limit = self.turn_vel_limit - (1 - angular_dist_from_origin/self.ramp_turn_vel_at_angle_default)*(self.turn_vel_limit - self.ramp_min_turn_vel_default)
            else:    
#                 rospy.loginfo(" INSIDE TURN() BLOCK 5 ")
                origin_limit = self.turn_vel_limit
            origin_limit = self.turn_vel_limit
            
            # select lowest velocity
            if origin_limit < limit:
#                 rospy.loginfo(" INSIDE TURN() BLOCK 6 ")
                limit = origin_limit
            if self.angular_vel > limit:
#                 rospy.loginfo(" INSIDE TURN() BLOCK 7")
                self.angular_vel = limit
            elif self.angular_vel < -limit:
#                 rospy.loginfo(" INSIDE TURN() BLOCK 8 ")
                self.angular_vel = -limit

            self.linear_vel = 0.0

    # return true if we have arrived at the destination (b) waypoint
    def arrived_at_waypoint(self):
        arrived = False
        
#         rospy.loginfo(" Checks if arrived at waypoint")
        if self.dist <= self.wpt_tolerance: # if we are inside the acceptable tolerance perimeter
            if self.dist < 0.01: # if we are VERY close.
                arrived = True
            elif self.dist > self.dist_minimum: # if we are moving away so we won't get any closer without turning.
                arrived = True
            elif fabs(self.angle_diff (self.heading_err, self.heading_err_prev)) > pi/8.0: # if large bearing jump
                arrived = True
        self.arr = arrived
        return arrived



    # calculate distance, bearing, heading error
    def update_navigation_state(self):
#             rospy.loginfo(" INSIDE UPDATE_NAVIGATION_STATE() ")
            self.debug_count+=1
            self.dist = sqrt((self.b[self.W_E]-self.pose[self.W_E])**2 + (self.b[self.W_N]-self.pose[self.W_N])**2) # dist to destination this is diff from the ab line. This dist is the value of the robot to the final destination from its current pose. The ab length is the dist between the waypoints that need to be navigated.
            self.dist_start = sqrt((self.start[self.W_E]-self.pose[self.W_E])**2 + (self.start[self.W_N]-self.pose[self.W_N])**2) # dist to starting position# initially dist_start is 0 as robot is starting at start position
            if self.dist<self.dist_minimum:
                self.dist_minimum = self.dist
            easting_component = self.b[self.W_E] - self.pose[self.W_E]# destination bearing (angle with easting axis)
            northing_component = self.b[self.W_N] - self.pose[self.W_N]
            self.bearing = self.angle_limit(atan2 (northing_component, easting_component)) 
            self.heading_err_prev = self.heading_err
            self.heading_err = self.angle_diff(self.bearing, self.pose[2]) # the diffeernce between the bearing angle and the current angle of the robot
     
            self.target_dist = self.dist
            self.target=self.b
            self.target_bearing = self.bearing
            if self.target_dist > self.dist:
                self.target_dist = self.dist
                self.target_bearing = self.bearing

            
           # rospy.loginfo(" target = "+str(self.target)+" target_dist = "+str(self.target_dist))
                 
             
            self.target_heading_err = self.angle_diff(self.target_bearing, self.pose[2])
            
            if self.debug_count==57:
                self.debug_count=0
#                 rospy.loginfo("Ishan Alg Coordinates are : "+"pose = "+str(self.pose))
       
    # make sure we don't exceed maximum linear and angular velocity
    def limit_vel(self):
       # rospy.loginfo(" Inside limit _vel. Limits the velocities")
        if self.linear_vel > self.max_linear_vel:
            self.linear_vel = self.max_linear_vel
        elif self.linear_vel < -self.max_linear_vel:
            self.linear_vel = -self.max_linear_vel

        # perform angular velocity limit
        if self.angular_vel > self.angular_vel_limit:
            self.angular_vel = self.angular_vel_limit
        elif self.angular_vel < -self.angular_vel_limit:
            self.angular_vel = -self.angular_vel_limit
        
#         rospy.loginfo(" Linear Velocity = "+str(self.linear_vel))
        
    # waypoint navigation updater, returns status, and linear and angular velocity to the controller. Comes here from updater function
    def update(self, time_stamp):
        status = self.UPDATE_NONE
        if self.state != self.STATE_STOP and self.pose != False: # if we have a valid pose

            # calculate distance, bearing, heading error
            self.update_navigation_state()
            curr_time = rospy.Time.now().to_sec()
            time_elapsed = curr_time-self.prev_time
#             rospy.loginfo("Time elapsed is "+str(time_elapsed)+" seconds")
            if(time_elapsed>=2):
#                 rospy.loginfo("Time elapsed is "+str(time_elapsed)+" seconds")
                time_elapsed=0
                self.prev_time=rospy.Time.now().to_sec()
                if(self.arr):
                    self.logData(" Arrived \n \n")
                    self.arr=False
                else:
                    self.logData(" ")

            # are we there yet?
            if self.arrived_at_waypoint():
                status = self.UPDATE_ARRIVAL
            
            else:
                if self.debug:
#                         print "%.3f  state %d dist %6.2f ab_dist %.2f  bearing %5.1f  t_dist %5.2f t_head_err %5.1f  lin_v %5.2f ang_v %5.2f" % (time_stamp, self.state, self.dist, self.ab_dist_to_pose, self.bearing*self.rad_to_deg, self.target_dist, self.target_heading_err*self.rad_to_deg, self.linear_vel, self.angular_vel)
                        self.debug_time_stamp = time_stamp

                # run navigation state machine    
                if self.state == self.STATE_DRIVE:
                    self.drive()# adjusts the velocity of the robot while driving forward
                elif self.state == self.STATE_TURN:
                    self.turn()

                self.limit_vel()
        return (status, self.linear_vel, self.angular_vel)
    
    # return angle within [0;2pi[
    def angle_limit (self, angle):
        while angle < 0:
            angle += self.pi2
        while angle >= self.pi2:
            angle -= self.pi2
        return angle
        
    # return signed difference between new and old angle
    def angle_diff (self, angle_new, angle_old):
        diff = angle_new - angle_old
        while diff < -pi:
            diff += self.pi2
        while diff > pi:
            diff -= self.pi2
        return diff
