#!/usr/bin/env python
  
import rospy
from sensor_msgs.msg import PointCloud2,PointCloud
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32

class lidarPaser:
    def __init__(self):
        rospy.init_node('lidar', anonymous=True)
    
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.pc1_pub = rospy.Publisher('/pc2',PointCloud2, queue_size=1)
        
        
        rospy.spin()

    def callback(self, data):
        pc2_msg=PointCloud2()
        pc2_msg = data

        pc2_msg.header.frame_id='gps'
        pc2_msg.header.stamp=data.header.stamp
        self.pc1_pub.publish(pc2_msg)
        # print("point num : {}".format(len(pc2_msg.points)))
        
        

if __name__ == '__main__':
    try:
        lidar = lidarPaser()
    except rospy.ROSInterruptException:
        pass