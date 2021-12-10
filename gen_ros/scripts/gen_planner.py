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

        arg = rospy.myargv(argv=sys.argv)
        self.map_name = "R_KR_PR_Naverlabs_Pangyo"
        self.op_case = arg[1]
        self.traffic_control=arg[2]

        if(self.op_case == "1"):
            self.destination = Point(605.714904785,-430.723876953,-2.04169845581)
        elif(self.op_case == "2"):
            self.destination = Point(608.984741211,-431.287658691,-2.07680082321)
            self.stop_over_point = [584.901489258, -545.642333984]
            check_npc_link = "6139"
        elif(self.op_case == "3"):
            self.destination = Point(814.503173828, -550.100036621, -2.99442005157)
            self.stop_over_point = [571.456359863, -546.48034668]
            check_npc_link = "6129"
        elif(self.op_case == "5" or self.op_case == "4"):
            self.destination =  Point(590.271972656, 413.530456543, 5.77461624146)# Point(-291.608154297, 600.307556152, 5.58355808258)
            self.stop_over_point = [619.141784668, 323.986297607]
            check_npc_link = "4169"
        elif(self.op_case == "6" or self.op_case == "7"):
            self.destination = Point(614.503662109, -12.0273742676, 0.753755569458)
        elif(self.op_case == "8"):
            self.destination = Point(615.366333008, -270.371154785, -2.64361953735)
            self.stop_over_point = [615.672119141, -430.026550293]
            check_npc_link = "5371"
        elif(self.op_case == "9"):
            self.destination = Point(581.772705078, -773.739135742, 1.8105700016)

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        traffic_pub = rospy.Publisher("/SetTrafficLight",SetTrafficLight,queue_size=1) ## TrafficLight Control
        
        ctrl_msg= CtrlCmd()
        odom_pub= rospy.Publisher('/odom',Odometry,queue_size=1)
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber

        
            
        # service

        rospy.wait_for_service('/Morai_TLSrv')
        self.tl_info_srv = rospy.ServiceProxy('Morai_TLSrv', MoraiTLInfoSrv)

        #def
        self.is_status=False ## 차량 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.is_traffic=False ## 신호등 상태 점검

        if(self.map_name == "R_KR_PR_Naverlabs_Pangyo"):
            self.traffic_info = [[555.016601562, -542.70690918, 'C119AS305185'],
                                [48.7231750488,-67.2655029297 ,'C119AS305053'],
                                [264.742156982,326.365142822 ,'C119AS305086']]
        else :
            self.traffic_info = [[58.50 , 1180.41, 'C119BS010001'], ## TrafficLight information
                                [85.61,1227.88 ,'C119BS010021'],
                                [136.58,1351.98 ,'C119BS010025'],
                                [141.02,1458.27 ,'C119BS010028'],
                                [139.39,1596.44 ,'C119BS010033']]

        while(not self.is_status):
            pass
        

        self.global_path_planner = globalPathPlanning(self.destination,self.map_name)
        self.global_path_planner.set_ego_status(self.status_msg)
        self.global_path = self.global_path_planner.calc_dijkstra_path() ## 출력할 경로의 이름
        
        if(self.op_case == "5" or self.op_case == "4"):
            path_reader=pathReader('gen_ros') ## 경로 파일의 위치
            self.global_path=path_reader.read_txt("op_case_5.txt")
        

        if(self.op_case == "8"):
            path_reader=pathReader('gen_ros') ## 경로 파일의 위치
            self.global_path=path_reader.read_txt("op_case_8.txt")

        #class
        pure_pursuit=purePursuit() ## purePursuit import
        pid=pidController()
        self.cc=cruiseControl(0.5,1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo=vaildObject(self.traffic_info) ## 장애물 유무 확인 (TrafficLight)
        base_velocity = 60 / 3.6
        if(self.op_case == "5" or self.op_case == "4" or self.op_case == "8"):
            base_velocity = 36 / 3.6
        if(self.op_case == "3" or self.op_case == "2" ):
            base_velocity = 60 / 3.6

        vel_planner=velocityPlanning(base_velocity,0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,50)



        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            if self.is_status==True and self.is_obj ==True:
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)

                self.vo.get_object(self.object_num,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3])
                global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])
                self.check_traffic()
                if self.is_traffic == True: ## 신호등 상태 점검 
                    if self.traffic_control == "True": ## 신호등 신호 초록색으로 변경(True) or (False)
                        self.tl_msg.trafficLightStatus=16 

                        ## traffic_control ##
                        self.set_traffic_data= SetTrafficLight()
                        self.set_traffic_data.trafficLightIndex = self.tl_msg.trafficLightIndex
                        self.set_traffic_data.trafficLightStatus = 16 ## 16 = Green light
                        ## traffic_control ##

                        # traffic_pub.publish(self.set_traffic_data) ## 차량 신호 Green Light 변경
                    self.cc.checkObject(local_path,global_obj,local_obj,[self.tl_msg.trafficLightIndex,self.tl_msg.trafficLightStatus]) 
                else :
                    self.cc.checkObject(local_path,global_obj,local_obj)
                # pure pursuit control
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                ctrl_msg.steering=-pure_pursuit.steering_angle()/180*pi
        

                cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg.position) ## advanced cruise control 적용한 속도 계획
                target_velocity = cc_vel

  

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


                if(self.op_case == "2" or self.op_case == "3" or self.op_case == "5" or self.op_case == "4" or self.op_case == "8"):
                    wait_dist = self.calc_dist_with_wp([local_path.poses[0].pose.position.x, local_path.poses[0].pose.position.y] , self.stop_over_point )
                    print("wait_dist : ", wait_dist)
                    if(wait_dist < 5 or ((self.op_case == "8" or self.op_case == "3" or self.op_case == "2") and wait_dist < 10)):
                        for npc in self.npc_msg:
                            # link_idx = self.global_path_planner.find_pos_link_idx(npc.position.x, npc.position.y, True)
                            # print(link_idx)
                            if (self.global_path_planner.is_near_link_idx(npc.position.x, npc.position.y, check_npc_link)):# (check_npc_link):
                                print("brake")            
                                ctrl_msg.accel= 0
                                ctrl_msg.brake= 1
                                break



                local_path_pub.publish(local_path) ## Local Path 출력
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력
                odom_pub.publish(self.makeOdomMsg())
                
            
                if count==30 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    count=0
                count+=1
                rate.sleep()



    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True


    def makeOdomMsg(self):
        odom=Odometry()
        odom.header.frame_id='map'
        odom.child_frame_id='gps'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(self.status_msg.heading))

        odom.pose.pose.position.x=self.status_msg.position.x
        odom.pose.pose.position.y=self.status_msg.position.y
        odom.pose.pose.position.z=self.status_msg.position.z
        odom.pose.pose.orientation.x=quaternion[0]
        odom.pose.pose.orientation.y=quaternion[1]
        odom.pose.pose.orientation.z=quaternion[2]
        odom.pose.pose.orientation.w=quaternion[3]


        return odom

    def objectInfoCB(self,data): ## Object information Subscriber
        self.object_num=data.num_of_npcs+data.num_of_obstacle+data.num_of_pedestrian
        self.npc_msg = data.npc_list
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        for num in range(data.num_of_obstacle) :
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian) :
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity.x)

        self.object_info=[object_type,object_pose_x,object_pose_y,object_velocity]
        self.is_obj=True

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
    
if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass
