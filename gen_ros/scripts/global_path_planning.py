#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from std_msgs.msg import Int32, String
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

# from factor import * 
from lib.mgeo.class_defs import *
# from d_dijkstra_no_lc import Dijkstra
from e_dijkstra import Dijkstra
from math import sqrt

class globalPathPlanning :

    def __init__(self,destination ,map_name):

        self.destination = destination

        self.links = None

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/'+map_name))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()
        self.global_planner=Dijkstra(self.nodes,self.links)


        # print('# of nodes: ', len(node_set.nodes))
        # print('# of links: ', len(link_set.lines))

 
    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=link_point[2]
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node

    def set_ego_status(self,msg):
        self.x = msg.position.x
        self.y = msg.position.y

    def find_ego_link_path(self):
        min_dis = float('inf')
        self.idx = None
        self.current_idx = None
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:                
                x = link_point[0]
                y = link_point[1]

                dx = self.x - x
                dy = self.y - y

                dist = sqrt(pow(dx,2)+pow(dy,2))         
                if dist < min_dis:
                    min_dis = dist
                    self.idx = link_idx        
        
        return self.idx

    def find_pos_link_idx(self,pos_x,pos_y,only_main_link = False):
        min_dis = float('inf')
        pos_idx = None
        for link_idx in self.links :
            if(only_main_link):
                if(len(self.links[link_idx].points) == 2):
                    continue
            for link_point in self.links[link_idx].points:                
                x = link_point[0]
                y = link_point[1]

                dx = pos_x - x
                dy = pos_y - y

                dist = sqrt(pow(dx,2)+pow(dy,2))         
                if dist < min_dis:
                    min_dis = dist
                    pos_idx = link_idx        
        
        return pos_idx
    
    def is_near_link_idx (self,pos_x,pos_y,link_idx):

        for link_point in self.links[link_idx].points:                
            x = link_point[0]
            y = link_point[1]

            dx = pos_x - x
            dy = pos_y - y

            dist = sqrt(pow(dx,2)+pow(dy,2))         
            if dist < 1:
                return True
        return False

    def calc_simple_global_path(self):
        x = self.x
        y = self.y
        goal_x=self.destination.x
        goal_y=self.destination.y
        # print(goal_x,goal_y)
        start_min_dis=float('inf')
        goal_min_dis=float('inf')


        for node_idx in self.nodes:
            node_pose_x=self.nodes[node_idx].point[0]
            node_pose_y=self.nodes[node_idx].point[1]
            start_dis=sqrt(pow(x-node_pose_x,2)+pow(y-node_pose_y,2))
            goal_dis=sqrt(pow(goal_x-node_pose_x,2)+pow(goal_y-node_pose_y,2))
            if start_dis < start_min_dis :
                start_min_dis=start_dis
                start_node_idx=node_idx
            if goal_dis < goal_min_dis :
                goal_min_dis=goal_dis
                end_node_idx=node_idx

        ego_link_idx = self.find_ego_link_path()
        start_node_idx = self.links[ego_link_idx].from_node.idx


    def calc_dijkstra_path(self):
        x = self.x
        y = self.y
        goal_x=self.destination.x
        goal_y=self.destination.y
        # print(goal_x,goal_y)
        start_min_dis=float('inf')
        goal_min_dis=float('inf')


        # for node_idx in self.nodes:
        #     node_pose_x=self.nodes[node_idx].point[0]
        #     node_pose_y=self.nodes[node_idx].point[1]
        #     start_dis=sqrt(pow(x-node_pose_x,2)+pow(y-node_pose_y,2))
        #     goal_dis=sqrt(pow(goal_x-node_pose_x,2)+pow(goal_y-node_pose_y,2))
        #     if start_dis < start_min_dis :
        #         start_min_dis=start_dis
        #         start_node_idx=node_idx
        #     if goal_dis < goal_min_dis :
        #         goal_min_dis=goal_dis
        #         end_node_idx=node_idx

        ego_link_idx = self.find_ego_link_path()
        print(ego_link_idx)
        start_node_idx = self.links[ego_link_idx].from_node.idx
        end_link_idx = self.find_pos_link_idx(goal_x, goal_y)
        end_node_idx = self.links[end_link_idx].to_node.idx
        result, path = self.global_planner.find_shortest_path(start_node_idx, end_node_idx)
        dijkstra_path=Path()
        dijkstra_path.header.frame_id='map'
        for waypoint in path["point_path"] :
            tmp_point=PoseStamped()
            tmp_point.pose.position.x=waypoint[0]
            tmp_point.pose.position.y=waypoint[1]
            dijkstra_path.poses.append(tmp_point)
            print(waypoint[0],waypoint[1])

        
        
        return dijkstra_path


if __name__ == '__main__':
    
    test_track=globalPathPlanning()



