#!/usr/bin/env python

'''
03-12-2014 IG Created First Time. This file logs the results obtained from the tests of the robot
into a log file
'''

import rospy
from sys import argv
import time

class LogResults():
    def __init__(self,id,param1, param2,param3,param4,param5,param6,param7,param8,param9,param10,
                 param11,param12,param13,param14,param15,param16,param17):
        self.id = id
        self.f=0
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.param5 = param5
        self.param6 = param6
        self.param7 = param7
        self.param8 = param8
        self.param9 = param9
        self.param10 = param10
        self.param11 = param11
        self.param12 = param12
        self.param13= param13
        self.param14 = param14
        self.param15 = param15
        self.param16 = param16
        self.param17 = param17
        self.filename = "/home/ishan/ish_frobomind/src/fmApp/ishan_surveying/ishan_surveying/LogResults/surveyor_logfile_strangeturn 16thJanuary Horsens.txt"
        rospy.loginfo("Logging Results")
        self.alg_name = ""
        self.tab_space = "\t\t\t\t\t\t"
        if(self.id==0):
            self.alg_name="Simple Algorithm"
        else:
            self.alg_name="Kjeld's Algorithm"
        self.file_open()
        self.f.write(" \n "+self.alg_name+" Results.\n"+time.strftime("%c")+"\n"+" Time "+"\n")
#                      self.tab_space+self.tab_space+param1+self.tab_space+param2+self.tab_space+param3+self.tab_space+param4+self.tab_space+param5+self.tab_space
#                      +param6+self.tab_space+param7+self.tab_space+param8+self.tab_space+param9+self.tab_space+
#                      param10+self.tab_space+param11+self.tab_space+param12+self.tab_space+param13+
#                      self.tab_space+param14+self.tab_space+param15+self.tab_space+param16+self.tab_space
#                      +param17+"\n")
        self.file_close()
    
    def file_open(self):
        self.f = open(self.filename,"a")
        
    def file_close(self):
        self.f.close()
    

    
    def log_results(self,heading_err,linear_vel,angular_vel,dist,posex,posey,posez,ablength,error,
                    status,ab_dist_to_pose,dee,point0,point1,target0,target1,target_heading_error):
        
#         self.f.write(time.strftime("%c")+"\n"+self.tab_space+str(heading_err)+self.tab_space+str(linear_vel)+
#                      self.tab_space+str(angular_vel)+self.tab_space+str(dist)+self.tab_space+
#                      str(posex)+self.tab_space+str(posey)+self.tab_space+str(posez)+self.tab_space
#                      +str(ablength)+self.tab_space+str(error)+self.tab_space+str(status)+
#                      self.tab_space+str(ab_dist_to_pose)+self.tab_space+str(dee)+self.tab_space+
#                      str(point0)+self.tab_space+str(point1)+self.tab_space+str(target0)+self.tab_space+
#                      str(target1)+self.tab_space+str(target_heading_error)+"\n")
        self.f.write(time.strftime("%c")+"\n"+self.param1+"\n"+str(heading_err)+"\n"+self.param2+"\n"+
                     str(linear_vel)+"\n"+self.param3+"\n"+str(angular_vel)+"\n"+self.param4+"\n"+
                     str(dist)+"\n"+self.param5+"\n"+str(posex)+"\n"+self.param6+"\n"+str(posey)+
                     "\n"+self.param7+"\n"+str(posez)+"\n"+self.param8+"\n"+str(ablength)+"\n"+
                     self.param9+"\n"+str(error)+"\n"+self.param10+"\n"+status+"\n"+
                     self.param11+"\n"+str(ab_dist_to_pose)+"\n"+self.param12+"\n"+
                     str(dee)+"\n"+self.param13+"\n"+str(point0)+"\n"+self.param14+"\n"+
                     str(point1)+"\n"+self.param15+"\n"+str(target0)+"\n"+self.param16+"\n"+
                     str(target1)+"\n"+self.param17+"\n"+str(target_heading_error)+"\n"+"\n")
        
        
if __name__ == '__main__':
    LogResults()
    