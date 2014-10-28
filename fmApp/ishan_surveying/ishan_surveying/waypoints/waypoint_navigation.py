#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or withoutn
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#	  names of its contributors may be used to endorse or promote products
#	  derived from this software without specific prior written permission.
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

# imports
from math import pi, atan2, sqrt, fabs, cos
from differential_kinematics import differential_kinematics
from pid_controller import pid_controller
import rospy

class waypoint_navigation():
	# dissected S1
	def __init__(self, update_rate, wheel_dist, drive_kp, drive_ki, drive_kd, drive_ff, drive_max_output, turn_kp, turn_ki, turn_kd, 
				 turn_ff, turn_max_output, max_linear_vel, max_angular_vel, wpt_tol_default, wpt_def_drive_vel, wpt_def_turn_vel, 
				 turn_start_at_heading_err, turn_stop_at_heading_err, ramp_drive_vel_at_dist, ramp_min_drive_vel, ramp_turn_vel_at_angle,
				  ramp_min_turn_vel, stop_nav_at_dist, debug):

		# constants
		self.UPDATE_NONE = 0
		self.UPDATE_ARRIVAL = 1
		self.pi2 = 2.0*pi
		self.deg_to_rad = pi/180.0
		self.rad_to_deg = 180.0/pi
		self.debug_count=0
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
		self.start = False # Value whether robot should start or not. Initially false as robot is stopped
		self.a = False  # Starting waypoint for the robot
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
#		 if self.debug:
#			 self.debug_time_stamp = 0.0
#			 self.print_count = 0
	
	 # dissected
	# call whenever a new pose is available
	def state_update (self, easting, northing, yaw, vel):
		self.pose = [easting, northing, yaw]
		self.vel = vel
		
	  #dissected
	 #call to set a new navigation destination waypoint
	def navigate (self, destination, origin):# initially robot only has a destination. Comes here only when automode is requested first time. After that each time it goes to the update method.
		 self.b = destination # where we are going
		 rospy.loginfo("Inside navigate state. Comes here every time a new waypoint needs to be navigated. Destination is "+str(self.b))
		 
		 if origin != False:# initially there is no origin of the robot and it is false. So, initally the starting position of the robot is its pose itself.
			 self.a = origin
		 else:
			 self.a = self.pose	
 
		 self.start = self.pose # where we started driving from (updated in drive_init())
		 rospy.loginfo("Inside Navigate State.Initially when either a new waypoint is loaded or when the robot starts first time. Variable 'start' is equal to "+str(self.start))
 
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
 
		 # calculate ab-line properties (used by the drive function)
		 rospy.loginfo(" Inside Navigate State. Every time a new waypoint is loaded or robot starts first time then the parameters of the robot like wpt tolerance, linear velocity, angular velocity are initialized. ")
		 self.ab_len = sqrt((self.b[self.W_E]-self.a[self.W_E])**2 + (self.b[self.W_N]-self.a[self.W_N])**2) # length of ab line
		 self.ab_norm = [(self.b[self.W_E]-self.a[self.W_E])/self.ab_len,(self.b[self.W_N]-self.a[self.W_N])/self.ab_len] # normalized ab vector
		 self.a_dot_ab_norm = self.a[0]*self.ab_norm[0] + self.a[1]*self.ab_norm[1] # (vector defined by point a) dot (normalized ab vector) 
		 rospy.loginfo(" 'ab_len' = "+str(self.ab_len)+" 'ab_norm' "+str(self.ab_norm))
		 # reset state variables
		 self.dist_minimum = 20000000.0
 
		 self.update_navigation_state()
		 self.drive_init()
	
	 # dissected
	# call to stop navigating to the destination
	def stop (self):
#		 if self.debug:
#			 print "Stop"
		self.a = False
		self.b = False
		self.linear_vel = 0.0
		self.angular_vel = 0.0
		self.state = self.STATE_STOP
		
	 # dissected
	# call to temporaily stop navigating to the destination
	def standby (self):
#		 if self.debug:
#			 print "Navigation standby"
		self.linear_vel = 0.0
		self.angular_vel = 0.0
		self.prev_state = self.state
		self.state = self.STATE_STANDBY

	 # dissected
	# call to resume navigating to the destination
	def resume (self):
#		 if self.debug:
#			 print "Resuming navigation"
		self.start = self.pose
		self.state = self.prev_state
		
	 # dissected
	# initialize drive towards destination (b) waypoint
	def drive_init(self):
			rospy.loginfo("Driving towards wpt")
			self.start = self.pose #where we started driving from
			rospy.loginfo(" Inside drive_init method. start = "+str(self.start))
			self.pid_drive.reset()
			rospy.loginfo(" Inside drive_init method. PID is reset here.")
		#self.heading_err_minimum = self.pi2
			self.state = self.STATE_DRIVE
			rospy.loginfo(" Robot is in Drive State")
			self.update_navigation_state() # update navigation values immediately
			self.drive() # execute the update immediately
	
	
	# drive towards destination (b) waypoint
	def drive(self):
		if fabs(self.target_heading_err) > self.turn_start_at_heading_err and self.dist > self.wpt_tolerance_default:
			self.turn_init()
			rospy.loginfo(" Turn the robot as heading error is greater than the heading error of the robot at start and robot has not reached near the waypoint.")
		else:
			# determine angular speed

			if self.dist > self.stop_nav_at_dist:
				self.angular_vel = self.pid_drive.update (self.target_heading_err) # get controller output
				rospy.loginfo(" Inside drive method. update angular velocity based on target heading error. PID error is used for this purpose.")
				self.pid_status = self.pid_drive.latest_update_values()
			else:
				self.angular_vel = 0.0

			# determine linear speed
			self.ramp_dest = 1.0
			self.ramp_start = 1.0
			self.ramp_turn = 1.0

			if self.dist < self.wpt_ramp_drive_vel_at_dist: # close to destination not clear
				self.ramp_dest = self.dist/self.wpt_ramp_drive_vel_at_dist
				rospy.loginfo(" Inside drive method.This is time.  'ramp_dest' = "+str(self.ramp_dest))
			if self.dist_start < self.wpt_ramp_drive_vel_at_dist: # close to starting point not clear
				self.ramp_start = self.dist_start/self.wpt_ramp_drive_vel_at_dist # not clear
				rospy.loginfo(" Inside drive method. This is time. 'ramp_start' = "+str(self.ramp_start))
			wheel_spd  = fabs(self.wheel_dist*self.angular_vel) # [m/s]
			self.ramp_turn = (self.wpt_drive_vel - wheel_spd)/self.wpt_drive_vel
			rospy.loginfo(" Inside drive method. This is velocity.  'ramp_turn' = "+str(self.ramp_turn))

			#if self.target_dist < self.wpt_ramp_drive_vel_at_dist: # close to target
			#	ramp_target = self.target_dist/self.wpt_ramp_drive_vel_at_dist

			self.ramp = self.ramp_dest
			if self.ramp_start < self.ramp:
				self.ramp = self.ramp_start
			if self.ramp_turn < self.ramp:
				self.ramp = self.ramp_turn
			
			
			self.linear_vel = self.wpt_drive_vel - (1-self.ramp)*(self.wpt_drive_vel - self.wpt_ramp_min_drive_vel)
			
			rospy.loginfo("Ramp variables are still unclear. 'ramp' is "+str(self.ramp))
	
	  

	 # dissected
	# initialize turn about own center (not applicable to all robots)
	def turn_init(self):
#		 if self.debug:		
#			 print 'Turning to adjust heading error: %.1f' %(self.target_heading_err*self.rad_to_deg)
		self.pid_turn.reset ()
		self.turn_bearing_origin = self.pose[2]# is the starting bearing angle of the robot
		#self.heading_err_minimum = self.pi2
		self.state = self.STATE_TURN
		self.turn() # execute the update immediately

	 # dissected
	def limit_value (self, value, maximum):
		if value > maximum:
			value = maximum
		elif value < -maximum:
			value = -maximum
		return value

	 # dissected
	# turn about own center (not applicable to all robots)
	def turn(self):
		# check if we have turned enough
		rospy.loginfo(" Checks if turn is enough and sets the angular velocity for the turn.")
		angular_dist_to_target = fabs(self.target_heading_err)
		if angular_dist_to_target <= self.turn_acceptable_heading_err:
			self.drive_init()
		else:
			pid_error = self.limit_value (self.target_heading_err, pi/4.0)
			self.angular_vel = self.pid_turn.update (pid_error) # get controller output
			self.pid_status = self.pid_turn.latest_update_values()
			rospy.loginfo("Calculates pid error ")

			# ramp velocity if close to target
			if angular_dist_to_target < self.ramp_turn_vel_at_angle_default:
				limit = self.turn_vel_limit - (1 - angular_dist_to_target/self.ramp_turn_vel_at_angle_default)*(self.turn_vel_limit - self.ramp_min_turn_vel_default)
			else:
				limit = self.turn_vel_limit
		
			# ramp velocity if close to origin
			angular_dist_from_origin = fabs(self.angle_diff (self.pose[2], self.turn_bearing_origin))
			if angular_dist_from_origin < self.ramp_turn_vel_at_angle_default:
				origin_limit = self.turn_vel_limit - (1 - angular_dist_from_origin/self.ramp_turn_vel_at_angle_default)*(self.turn_vel_limit - self.ramp_min_turn_vel_default)
			else:	
				origin_limit = self.turn_vel_limit
			origin_limit = self.turn_vel_limit
			
			rospy.loginfo("Sets the ramp velocity of the robot. Unclear")
			# select lowest velocity
			if origin_limit < limit:
				limit = origin_limit
			if self.angular_vel > limit:
				self.angular_vel = limit
			elif self.angular_vel < -limit:
				self.angular_vel = -limit

			self.linear_vel = 0.0

	 # dissected
	# return true if we have arrived at the destination (b) waypoint
	def arrived_at_waypoint(self):
		arrived = False
		rospy.loginfo(" Checks if arrived at waypoint")
		if self.dist <= self.wpt_tolerance: # if we are inside the acceptable tolerance perimeter
			if self.dist < 0.01: # if we are VERY close.
				arrived = True
			elif self.dist > self.dist_minimum: # if we are moving away so we won't get any closer without turning.
				arrived = True
			elif fabs(self.angle_diff (self.heading_err, self.heading_err_prev)) > pi/8.0: # if large bearing jump
				arrived = True
		return arrived



	# dissected
	# calculate distance, bearing, heading error
	def update_navigation_state(self):
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
	 
			# find the point on the ab-line that is closest to the robot
			pose_dot_ab_norm = self.pose[self.W_E]*self.ab_norm[0] + self.pose[self.W_N]*self.ab_norm[1] # (vector defined by pose) dot (normalized ab vector)
			d = pose_dot_ab_norm - self.a_dot_ab_norm  # initially this is 0
	 
			pt = [self.a[0] + d*self.ab_norm[0], self.a[1] + d*self.ab_norm[1]] # closest point along the ab line from the robot's current pose
			self.ab_dist_to_pose = sqrt((self.pose[self.W_E]-pt[self.W_E])**2 + (self.pose[self.W_N]-pt[self.W_N])**2)#distance to that point from the robot's current pose
			
			rospy.loginfo(" Inside update_navigation_state. 'dist' = "+str(self.dist)+" dist_start = "+str(self.dist_start)+" d = "+str(d)+" pt  = "+str(pt)+" ab_dist_to_pose = "+str(self.ab_dist_to_pose)+" a = "+str(self.a))
			
			if d > self.ab_len: # the pose lies beyond the end of the ab line and the closest point is therefore in fact b
				rospy.loginfo(" Pose is beyond ab line")
				self.target = self.b # set b as target point	
				self.target_bearing = self.bearing#set the bearing on reaching the target
				self.target_dist = self.dist
				rospy.loginfo(" target = "+str(self.target)+" target_dist = "+str(self.target_dist))
				'''elif d < 0: # the pose lies before the beginning of the ab line so the closest point is in fact a
				#self.target = [self.a[self.W_E], self.a[self.W_E]]
				#self.target_bearing = self.angle_limit(atan2 (self.target[self.W_N]-self.pose[self.W_N], self.target[self.W_E]-self.pose[self.W_E])) #  target bearing (angle with easting axis)			
				#self.target_dist = sqrt((self.target[self.W_E]-self.pose[self.W_E])**2 + (self.target[self.W_N]-self.pose[self.W_N])**2) # dist to target
	 
				self.target = self.b # set b as target point	
				self.target_bearing = self.bearing
				self.target_dist = self.dist 
				'''
			else: # the closest point is between A and B
				if self.ab_dist_to_pose < 0.001:# if the distance to closest point is less than 0.001 then robot is very close to it and hence the target will be b 
					rospy.loginfo(" Robot is very near to the closest point on ab line")
					self.target = self.b # set b as target point	
					self.target_dist = self.dist
					self.target_bearing = self.bearing
					rospy.loginfo(" target = "+str(self.target)+" target_dist = "+str(self.target_dist))
				else:
					# determine target point based on robot heading and dist from ab line
					rospy.loginfo(" Closest point somewhere between a and b points on ab line")
					ab_e = self.b[self.W_E] - self.a[self.W_E]  
					ab_n = self.b[self.W_N] - self.a[self.W_N]
					self.ab_bearing = self.angle_limit(atan2 (ab_n, ab_e)) # ab line bearing (angle with easting axis)
					self.left_of_ab = (self.angle_diff(self.bearing, self.ab_bearing) < 0.0)# calculates the diff between the bearing of the robot and the bearing of the ab line and if this is less than 0 degrees then the robot is to the left of ab line
	 
					self.target_max_angle = pi/6.0 
					self.target_angle_at_1m_offset = 90*pi/180.0
				 
					offset_angle = self.ab_dist_to_pose**1.2 * self.target_angle_at_1m_offset
					if offset_angle > self.target_max_angle:
						offset_angle = self.target_max_angle
					if self.left_of_ab: # the robot pose is to the left of the ab line
						offset_angle_signed = - offset_angle
						rospy.loginfo(" The robot is to the left of ab line. "+" left of ab line = "+str(self.left_of_ab))
					else:
						offset_angle_signed = offset_angle
	 
					self.target_bearing = self.angle_limit(self.ab_bearing + offset_angle_signed)
					self.target_dist = self.ab_dist_to_pose/cos(pi/2.0 - offset_angle) # dist to target, OBS assuming offset_angle != 0	
					if self.target_dist > self.dist:
						self.target_dist = self.dist
						self.target_bearing = self.bearing
					self.dist_pt_to_target = sqrt (self.target_dist**2 - self.ab_dist_to_pose**2)# subtracts the distance of the target from the robot's current pose and the distance of the robot to the closest point. So, the result is the distance of the closest point on the ab line from the robot to the target point
					self.target = [pt[self.W_E] + self.dist_pt_to_target*self.ab_norm[0], pt[self.W_N] + self.dist_pt_to_target*self.ab_norm[1]]# waypoint of the target point depending upon the values calculated above of the robots current pose and the closest point on the ab line
			
			rospy.loginfo(" target = "+str(self.target)+" target_dist = "+str(self.target_dist))
				 
			 
			self.target_heading_err = self.angle_diff(self.target_bearing, self.pose[2])
			# signed diff. heading to target bearing


		#print self.target_dist, self.target_heading_err, self.a[0], self.a[1], self.b[self.W_E], self.b[self.W_N], self.pose[0], self.pose[1], self.target[0], self.target[1] 

	 # dissected
	# make sure we don't exceed maximum linear and angular velocity
	def limit_vel(self):
		rospy.loginfo(" Inside limit _vel. Limits the velocities")
		if self.linear_vel > self.max_linear_vel:
			self.linear_vel = self.max_linear_vel
		elif self.linear_vel < -self.max_linear_vel:
			self.linear_vel = -self.max_linear_vel

		# perform angular velocity limit
		if self.angular_vel > self.angular_vel_limit:
			self.angular_vel = self.angular_vel_limit
		elif self.angular_vel < -self.angular_vel_limit:
			self.angular_vel = -self.angular_vel_limit
	
	 # dissected
	# waypoint navigation updater, returns status, and linear and angular velocity to the controller. Comes here from updater function
	def update(self, time_stamp):
		rospy.loginfo(" Inside update method. Comes here evrytime when the robot is in the process of navigating between one wpt to another."+" Total iterations is "+str(self.debug_count))
		status = self.UPDATE_NONE
		if self.state != self.STATE_STOP and self.pose != False: # if we have a valid pose

			# calculate distance, bearing, heading error
			self.update_navigation_state()

			# are we there yet?
			if self.arrived_at_waypoint():
				status = self.UPDATE_ARRIVAL
				rospy.loginfo("Total iterations are "+str(self.debug_count))
			
			else:
				# print result (debug)
				if self.debug:
#                     self.print_count += 1
# 					if self.print_count % self.print_interval == 0:
						print "%.3f  state %d dist %6.2f ab_dist %.2f  bearing %5.1f  t_dist %5.2f t_head_err %5.1f  lin_v %5.2f ang_v %5.2f" % (time_stamp, self.state, self.dist, self.ab_dist_to_pose, self.bearing*self.rad_to_deg, self.target_dist, self.target_heading_err*self.rad_to_deg, self.linear_vel, self.angular_vel)
						self.debug_time_stamp = time_stamp

				# run navigation state machine	
				if self.state == self.STATE_DRIVE:
					rospy.loginfo("Inside uopdate method. Drive the robot")
					self.drive()# adjusts the velocity of the robot while driving forward
				elif self.state == self.STATE_TURN:
					rospy.loginfo(" Inside update method. Turn the robot")
					self.turn()

				self.limit_vel()
		return (status, self.linear_vel, self.angular_vel)
	
	 # dissected
	# return angle within [0;2pi[
	def angle_limit (self, angle):
		while angle < 0:
			angle += self.pi2
		while angle >= self.pi2:
			angle -= self.pi2
		return angle
		
	 # dissected
	# return signed difference between new and old angle
	def angle_diff (self, angle_new, angle_old):
		diff = angle_new - angle_old
		while diff < -pi:
			diff += self.pi2
		while diff > pi:
			diff -= self.pi2
		return diff
