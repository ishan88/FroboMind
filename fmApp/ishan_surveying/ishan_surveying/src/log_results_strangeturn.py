#!/usr/bin/env python

'''
03-12-2014 IG Created First Time. This file logs the results obtained from the tests of the robot
into a log file
'''

import rospy
from sys import argv
import time

class LogResultsStrangeTurn():
    def __init__(self,id,Kp,Ki,Kd,ff):
        self.filename = "/home/ishan/ish_frobomind/src/fmApp/ishan_surveying/ishan_surveying/LogResults/surveyor_logfile_strangeturn_realrobot_pidtuning 16thJan Horsens.txt"
        rospy.loginfo("Logging Results")
        self.alg_name = ""
        if(id==0):
            self.alg_name="Simple Algorithm"
        else:
            self.alg_name="Kjeld's Algorithm"
        self.file_open()
        self.f.write(" \n "+self.alg_name+" Results.\n"+time.strftime("%c")+"\n"+"Kp: "+str(Kp)+"\n"+"Ki: "+str(Ki)+
                     "\n"+"Kd: "+str(Kd)+"\n"+"ff: "+str(ff)+"\n")
        self.file_close()
    
    def file_open(self):
        self.f = open(self.filename,"a")
        
    def file_close(self):
        self.f.close()
    
    def log_results_strangeturn(self,data):
        self.f.write(data+"\n")
        
        
        
if __name__ == '__main__':
    LogResultsStrangeTurn()
    