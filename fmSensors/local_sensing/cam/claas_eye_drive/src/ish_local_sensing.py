#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class LocalSensing():
    def __init__(self):
        
        
        rospy.init_node('ish_local_sensing', anonymous=False)
        rospy.loginfo(" Local Sensing Module started ")
        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(1)
        self.locSensTopic = rospy.get_param("~localsensingdata", '/fmInformation/locSenDat')
        
        self.pub = rospy.Publisher(self.locSensTopic,String)
        rospy.sleep(1)
       
        # rospy.Publisher('localsensingdata', String, queue_size=10)
               
        while not rospy.is_shutdown():
            str = "LocalSensingModule Data"
            self.pub.publish(str)
            r.sleep()
            
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)    
            

if __name__ == '__main__':
    try:
        LocalSensing()
    except:
        rospy.loginfo("Node Terminated")
        
