#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class FaultDiagnosis():
    def __init__(self):
        
        rospy.init_node('ish_fault_diagnosis', anonymous=False)
        
        rospy.loginfo("Fault Diagnosis Node Started")
        
        self.faultDiagnosisTopic = rospy.get_param("~faultdiagnosisdata", '/fmSafety/fauDiaDat')
        self.platfFeedbackTopic = rospy.get_param("~platformfeedbackdata", '/fmInformation/plaFeeDat')
        self.impFeedbackTopic = rospy.get_param("~implementfeedbackdata", '/fmInformation/impFeeDat')
        
        
        self.pub = rospy.Publisher(self.faultDiagnosisTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        
        rospy.Subscriber(self.platfFeedbackTopic, String, self.on_platfFeedbackTopic)
        rospy.Subscriber(self.impFeedbackTopic, String, self.on_impFeedbackTopic)
        
        while not rospy.is_shutdown():
            str = "IncidentHandlerData"
            self.pub.publish(str)
            r.sleep()
        
    def on_platfFeedbackTopic(self, msg): 
        rospy.sleep(20)  
        
    def on_impFeedbackTopic(self, msg): 
        rospy.sleep(20)   

if __name__ == '__main__':
    try:
        FaultDiagnosis()
        rospy.spin()
    except:
        rospy.loginfo("FaultDiagnosis Node Terminated")
        
        
        
        
        
        
        
        
        