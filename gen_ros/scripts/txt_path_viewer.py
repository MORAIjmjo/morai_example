#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight , MoraiTLIndex ,MoraiTLInfo

from morai_msgs.srv import MoraiTLInfoSrv
from lib.utils import pathReader, findLocalPath,purePursuit,pidController,velocityPlanning,vaildObject,cruiseControl
import tf
from math import cos,sin,sqrt,pow,atan2,pi
from global_path_planning import globalPathPlanning
class gen_planner():
    def __init__(self):
        rospy.init_node('gen_planner', anonymous=True)

        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher

        path_reader=pathReader('gen_ros') ## 경로 파일의 위치
        self.global_path=path_reader.read_txt("k-city.txt")
        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():

            
            if count==30 : ## global path 출력
                global_path_pub.publish(self.global_path)
                count=0
            count+=1
            rate.sleep()

    
if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass
