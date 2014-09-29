#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class MissionPlanner():
    def __init__(self):
        
        rospy.init_node('ish_mission_planner', anonymous=False)


        rospy.loginfo(" Mission Planner Node Started")
        
        rospy.on_shutdown(self.shutdown)
        
        
        self.monitorsTopic = rospy.get_param("~monitorsdata",'/fmDecisions/monDat')
        self.missionPlanTopic = rospy.get_param("~missionplannerdata", '/fmDecisions/misPlaDat')
        self.hmiTopic = rospy.get_param("~hmidata",'/fmDecisions/hmiDat')

        r = rospy.Rate(1)
        self.pub = rospy.Publisher(self.missionPlanTopic,String)
        rospy.sleep(1)
        
        rospy.Subscriber(self.monitorsTopic,String,self.on_monitorsTopic)
        rospy.Subscriber(self.hmiTopic, String, self.on_hmiTopic)
        
        while not rospy.is_shutdown():
            str = "MissionPlannerData"
            self.pub.publish(str)
            r.sleep()

    def on_missionPlanTopic(self,msg):
        rospy.sleep(20)
    
    def on_hmiTopic(self,msg):
        rospy.sleep(20)
    
    def on_monitorsTopic(self,msg):
        rospy.sleep(20)  
        
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)   
        
if __name__ == '__main__':
    try:
        MissionPlanner()
    except:
        rospy.loginfo("Mission Planner Node Terminated")