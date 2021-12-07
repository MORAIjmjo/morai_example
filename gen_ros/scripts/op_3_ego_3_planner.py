#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os, time
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
        rospy.init_node('ego_3', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.map_name = "R_KR_PR_Naverlabs_Pangyo"
        self.op_case = arg[1]
        self.traffic_control=arg[2]

        if(self.op_case == "3"):
            self.destination = Point(606.742431641,-378.698547363, -2.61392211914)

        #publisher
        ctrl_pub = rospy.Publisher('/ego_3/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        ctrl_msg= CtrlCmd()
        #subscriber
        rospy.Subscriber("/ego_3/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.egoCB) ## Vehicl Status Subscriber
        global_path_pub= rospy.Publisher('/ego_3/global_path',Path, queue_size=1) ## global_path publisher

        #def
        self.is_status=False ## 차량 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.is_traffic=False ## 신호등 상태 점검
        self.ego_status=False

        while(not self.is_status and not self.ego_status):
            pass
        

        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        self.global_path_planner = globalPathPlanning(self.destination,self.map_name)
        self.global_path_planner.set_ego_status(self.status_msg)
        self.global_path = self.global_path_planner.calc_dijkstra_path() ## 출력할 경로의 이름
        
        ego_local_dist = self.local_of_me(self.ego_msg, Point(566.443237305, -546.3179321291,-1.26130819321 ))
        while not rospy.is_shutdown() :
            if(ego_local_dist > 0):
                ego_local_dist = self.local_of_me(self.ego_msg,Point(566.443237305, -546.3179321291,-1.26130819321 ))
                ctrl_msg.accel= 0
                ctrl_msg.brake= 1
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력
                print(ego_local_dist)
                rate.sleep()
            else:
                break
        

        #class
        pure_pursuit=purePursuit() ## purePursuit import
        pid=pidController()
        target_velocity = 60 / 3.6

        while not rospy.is_shutdown():
            if self.is_status==True and self.ego_status == True:
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)

                # pure pursuit control
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                ctrl_msg.steering=-pure_pursuit.steering_angle()/180*pi

                control_input=pid.pid(target_velocity,self.status_msg.velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)

                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0
                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input

                if target_velocity < 1 and self.status_msg.velocity.x < 1 :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= 1

                if len(local_path.poses) < 15 :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= 1

                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력
                
                if count==30 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    count=0
                count+=1
                rate.sleep()




    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data
        # br = tf.TransformBroadcaster()
        # br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
        #                 tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
        #                 rospy.Time.now(),
        #                 "gps",
        #                 "map")
        self.is_status=True

    def egoCB(self,data): ## Vehicle Status Subscriber 
        self.ego_msg=data
        self.ego_status=True

    def calc_dist(self, tl):
        return sqrt(pow(self.status_msg.position.x - tl[0] ,2) + pow(self.status_msg.position.y - tl[1] ,2) )
    
    def check_traffic(self):
        for tl in self.traffic_info:
            if(self.calc_dist(tl) < 20):
                req_idx = MoraiTLIndex()
                req_idx.idx = tl[2]
                tl_info_resp = self.tl_info_srv(req_idx)
                self.tl_msg=GetTrafficLightStatus()
                self.tl_msg.trafficLightIndex = tl_info_resp.response.idx
                self.tl_msg.trafficLightStatus = tl_info_resp.response.status
                rospy.loginfo(self.tl_msg)
                self.is_traffic = True


    def calc_dist_with_wp(self,pos1 , pos2):
        return sqrt(pow(pos1[0] - pos2[0] ,2) + pow(pos1[1] - pos2[1] ,2))
        # self.is_traffic = False


    def local_of_me(self, origin_pos , target_pos):
        theta=origin_pos.heading/180*pi
        translation=[origin_pos.position.x, origin_pos.position.y ]
        rotation = np.array([[cos(theta), -sin(theta), 0], \
                             [sin(theta), cos(theta) , 0], \
                             [0 , 0, 1]])
        trans_rotat = np.array([[rotation[0][0], rotation[1][0], -(rotation[0][0]*translation[0] + rotation[1][0]* translation[1])], \
                                [rotation[0][1], rotation[1][1], -(rotation[0][1]*translation[0] + rotation[1][1]* translation[1])], \
                                [0,0, 1]])

        global_xy = np.array([[target_pos.x], [target_pos.y], [1]])
        local_xy = trans_rotat.dot(global_xy)
        
        local_target_x = local_xy[0][0]
        local_target_y = local_xy[1][0]
        local_target__z = 0

        return local_target_x


if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass
