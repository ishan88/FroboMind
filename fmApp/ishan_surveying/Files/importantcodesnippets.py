def turnBehavior(self):
        goal_angle = pi/2
        self.angular_vel = 0.2
        (position, rotation) = self.get_odom()
        last_angle = rotation
        turn_angle=0
         
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle):
            self.turn(self.angular_vel)
            (position, rotation) = self.get_odom()
            delta_angle = normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation
            rospy.loginfo("Turning the Robot"+"angle turned = "+str(turn_angle+self.angular_tolerance)+" goal_angle = "+str(goal_angle))
             
             
        self.stop()
#         while not rospy.is_shutdown() and self.stop_robot==False:
#             self.moveForward()
#         (position, rotation) = self.get_odom()
#         x_start = position.x
#         y_start = position.y
#         distance=0
#         
#         while distance<goal_distance and not rospy.is_shutdown():
#             
#             self.moveForward()
#             (position, rotation) = self.get_odom()
#             distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
#             rospy.loginfo("Distance travelled is "+str(distance)+" Goal Distance is "+str(goal_distance))
#             
#         self.stop()
#         rospy.sleep(3600)  