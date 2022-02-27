#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import numpy as np
import cv2, PIL
from cv2 import aruco
from std_msgs.msg import Int16
import roslib
import matplotlib as mpl
import pandas as pd
# import matplotlib.pyplot as plt
import math
import sys
import sys
import rospy
import cv2


import time

import itertools
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#카메라 위치는 리스트로 하나하나 매핑
class image_converter:
  
  def __init__(self):
   
    self.opencv2pose = PoseStamped()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("dji_osdk_ros/fpv_camera_images",Image,self.callback)
    self.marker_pose_pub = rospy.Publisher("vm_aruco_marker_pose",PoseStamped,queue_size=1)
    self.marker_pose = PoseStamped()
  def callback(self,data):
    # print("%f , %f"%(data.height,data.width))
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)


    #fpv
    real_L_length = 100.0
    width = 1280.0
    height =960.0

 
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners,ids,rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(),corners,ids)

        

    try:
        for i in range(len(ids)):
            c = corners[i][0]
       
        corners2 = np.array([c[0] for c in corners])

        data = pd.DataFrame({"x": corners2[:,:,0].flatten(), "y": corners2[:,:,1].flatten()},
                    index = pd.MultiIndex.from_product( 
                            [ids.flatten(), ["c{0}".format(i )for i in np.arange(4)+1]], 
                        names = ["marker", ""] ))


        



        data = data.unstack().swaplevel(0, 1, axis = 1).stack()
        data["m1"] = data[["c1", "c2"]].mean(axis = 1)
        data["m2"] = data[["c2", "c3"]].mean(axis = 1)
        data["m3"] = data[["c3", "c4"]].mean(axis = 1)
        data["m4"] = data[["c4", "c1"]].mean(axis = 1)
        data["o"] = data[["m1", "m2", "m3", "m4"]].mean(axis = 1)
        
        idx = pd.IndexSlice

        data.loc[idx[:,'x'],idx[:]] = (data.loc[idx[:,'x'],idx[:]]-width/2)
        data.loc[idx[:,'y'],idx[:]] = (data.loc[idx[:,'y'],idx[:]]-height/2)
        
        total_id_list=[]

        for i in range(len(ids)):
            total_id_list.append(ids[i][0])
            
        for i in range(len(total_id_list)):
            p1_x = data.loc[idx[total_id_list[i],'x'],idx['c1']]
            p1_y = data.loc[idx[total_id_list[i],'y'],idx['c1']]
            p2_x = data.loc[idx[total_id_list[i],'x'],idx['c2']]
            p2_y = data.loc[idx[total_id_list[i],'y'],idx['c2']]
            # p3_x = data.loc[idx[total_id_list[i],'x'],idx['c3']]
            # p3_y = data.loc[idx[total_id_list[i],'y'],idx['c3']]
            # p4_x = data.loc[idx[total_id_list[i],'x'],idx['c4']]
            # p4_y = data.loc[idx[total_id_list[i],'y'],idx['c4']]


            img_L_length =math.sqrt((p1_x-p2_x)**2 + (p1_y-p2_y)**2)   
            converting_const = real_L_length/img_L_length
            # print("%f %f"%(real_L_length,img_L_length))
            # yaw = math.acos()
            # print("%f"%(abs(p2_x-p1_x)/abs(p2_y-p1_y)))
            p1_p2_rot = math.atan2((p1_x-p2_x),(p1_y-p2_y))
            p1_p2_rot = math.degrees(p1_p2_rot) + 90
                    

            c1_o_x = (-1)*data.loc[idx[total_id_list[i],'x'],idx['o']]*converting_const 
            c1_o_y = data.loc[idx[total_id_list[i],'y'],idx['o']]*converting_const 
            # self.marker_pose.header.frame_id = str(total_id_list[i])
            self.marker_pose.header.frame_id = str(total_id_list[i])
            self.marker_pose.pose.position.x = c1_o_x
            self.marker_pose.pose.position.z = c1_o_y
            self.marker_pose.pose.position.y = (real_L_length/img_L_length)
            self.marker_pose_pub.publish(self.marker_pose)
            print("total_id_list : %d , pose_x : %f, pose_y : %f , rot : %f, "%(total_id_list[i],c1_o_x,c1_o_y,p1_p2_rot))
         
    except Exception as ex:
        print(ex)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)