#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os, time
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd,GetTrafficLightStatus,SetTrafficLight, SyncModeCmd, SyncModeCmdResponse, WaitForTick, WaitForTickResponse, EventInfo, SyncModeCtrlCmd
from morai_msgs.msg import SyncModeSetGear
from morai_msgs.srv import MoraiSyncModeCmdSrv ,MoraiWaitForTickSrv , MoraiEventCmdSrv ,MoraiScenarioLoadSrvRequest , MoraiSyncModeCtrlCmdSrv, MoraiSyncModeSetGearSrv
from lib.utils import pathReader, findLocalPath,purePursuit,pidController,velocityPlanning
import tf
from math import cos,sin,sqrt,pow,atan2,pi
import datetime

TIMES_STEP = 20
FIXED_DELTA_TIME = 0.02

class DataTotxt:

    def __init__(self, file_name, pkg_name):
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path +'/scripts/data/'+file_name+'.txt'
        self.f=open(full_path, 'w')
        print("Write path : ", file_name)

    def appendData(self , data):

        self.f.write(data)

    def closetxt(self):
        self.f.close()



class sync_planner():
    def __init__(self):

        ### write data
        curr_date = datetime.datetime.now()
        file_name = "sync_mode_fdt_02_" + str(curr_date.month) +str(curr_date.day) +str(curr_date.hour) +str(curr_date.minute)
        self.data_txt = DataTotxt(file_name, "method_ex") # "sensor_capture_data"
        ### write data

        rospy.init_node('sync_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        sensor_capture_mode=bool(arg[2])

        path_reader=pathReader('gen_ros') ## 경로 파일의 위치
        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher

        odom_pub= rospy.Publisher('/basic_odom',Odometry,queue_size=1)

        #service
        rospy.wait_for_service('/SyncModeCmd')
        rospy.wait_for_service('/SyncModeWaitForTick')
        rospy.wait_for_service('/SyncModeCtrlCmd')
        rospy.wait_for_service('/SyncModeSetGear')

        sync_mode_srv = rospy.ServiceProxy('SyncModeCmd', MoraiSyncModeCmdSrv)
        tick_wait_srv = rospy.ServiceProxy('SyncModeWaitForTick', MoraiWaitForTickSrv)
        ctrl_cmd_srv = rospy.ServiceProxy('SyncModeCtrlCmd', MoraiSyncModeCtrlCmdSrv)
        set_gear_srv = rospy.ServiceProxy('SyncModeSetGear',MoraiSyncModeSetGearSrv)

        #def
        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

        # set gear





        # def tick service
        sync_mode_on = SyncModeCmd()
        sync_mode_on.user_id = "sync_master"
        sync_mode_on.time_step = 20 # 20ms
        sync_mode_on.start_sync_mode = True

        frame_step = sync_mode_on.time_step / TIMES_STEP

        print("Synchronous Mode ON")
        sync_mode_resp = sync_mode_srv(sync_mode_on)
        self.next_frame = sync_mode_resp.response.frame + 1

        set_gear_cmd = SyncModeSetGear()
        set_gear_cmd.gear = 4
        set_gear_cmd.frame = self.next_frame
        set_gear_resp = set_gear_srv(set_gear_cmd)
        print(set_gear_resp.response)

        tick=WaitForTick()
        tick.user_id = sync_mode_resp.response.user_id
        tick.frame = self.next_frame
        tick_resp = tick_wait_srv(tick)



        # status 
        self.status_msg = tick_resp.response.vehicle_status
        self.tfBraodcaster()

        # control var
        ctrl_cmd = SyncModeCtrlCmd()
        ctrl_cmd.sensor_capture = sensor_capture_mode

        #class
        pure_pursuit=purePursuit() ## purePursuit import
        pid=pidController()
        vel_planner=velocityPlanning(60/3.6,0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)

        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        self.prev_x = self.status_msg.position.x
        self.prev_y = self.status_msg.position.y
        self.prev_time_stamp = 0.0
        self.accm_dist = 0.0
        self.current_time = 0.0


        print("Start Frame : ", self.next_frame + 1)        
        while not rospy.is_shutdown():
            try:
                print("send Tick!")
                ## global_path와 차량의 statu\s_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)

                # pure pursuit control
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                ctrl_cmd.command.steering = 0.0
                ctrl_cmd.command.accel = 0.25
                # ctrl_cmd.command.steering=-pure_pursuit.steering_angle()/180*pi
        
                # target_velocity = vel_profile[self.current_waypoint]

                # control_input=pid.pid(target_velocity,self.status_msg.velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)

                # if control_input > 0 :
                #     ctrl_cmd.command.accel = control_input
                #     ctrl_cmd.command.brake = 0
                # else :
                #     ctrl_cmd.command.accel = 0
                #     ctrl_cmd.command.brake = -control_input

                local_path_pub.publish(local_path) ## Local Path 출력
                odom_pub.publish(self.makeOdomMsg())
                
                self.next_frame += frame_step

                ctrl_cmd.frame = self.next_frame

                # send ctrl cmd                 
                ctrl_cmd_resp = ctrl_cmd_srv(ctrl_cmd)

                # Send Tick
                tick.frame = self.next_frame 
                tick_resp = tick_wait_srv(tick)
                if(self.current_time > 12):
                    self.data_txt.closetxt()
                    sync_mode_on.start_sync_mode = False
                    sync_mode_resp = sync_mode_srv(sync_mode_on)
                    # quit
                    sys.exit()
                self.status_msg = tick_resp.response.vehicle_status
                
                self.current_time += FIXED_DELTA_TIME
                self.accm_dist += sqrt(pow(self.status_msg.position.x - self.prev_x , 2) + pow(self.status_msg.position.y - self.prev_y, 2))
                dt = '{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\n'.format(\
                    self.current_time,\
                    tick_resp.response.vehicle_status.position.x,\
                    tick_resp.response.vehicle_status.position.y,
                    tick_resp.response.vehicle_status.velocity.x,\
                    tick_resp.response.vehicle_status.velocity.y, \
                    tick_resp.response.vehicle_status.acceleration.x,\
                    tick_resp.response.vehicle_status.acceleration.y,\
                    tick_resp.response.vehicle_status.accel,\
                    tick_resp.response.vehicle_status.brake ,\
                    tick_resp.response.vehicle_status.wheel_angle,\
                    ctrl_cmd.command.accel,\
                    ctrl_cmd.command.brake ,\
                    ctrl_cmd.command.steering,\
                    self.accm_dist)
                
                self.data_txt.appendData(dt)
                self.prev_x = self.status_msg.position.x
                self.prev_y = self.status_msg.position.y
        
                self.tfBraodcaster()

                if count==30 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    count=0
                count+=1

                rate.sleep()
            except KeyboardInterrupt:
                sync_mode_on.start_sync_mode = False
                sync_mode_resp = sync_mode_srv(sync_mode_on)
                # quit
                sys.exit()


    def calc_dist(self):
        print(self.current_time)
        self.current_time += FIXED_DELTA_TIME
        self.accm_dist += sqrt(pow(self.status_msg.position.x - self.prev_x , 2) + pow(self.status_msg.position.y - self.prev_y, 2))
        dt = '{0}\t{1}\n'.format(self.current_time , self.accm_dist)
        self.data_txt.appendData(dt)
        self.prev_x = self.status_msg.position.x
        self.prev_y = self.status_msg.position.y
        


    def tfBraodcaster(self): ## Vehicle Status Subscriber 
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")

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

if __name__ == '__main__':
    try:
        synchronous_planner=sync_planner()
    except rospy.ROSInterruptException:
        pass