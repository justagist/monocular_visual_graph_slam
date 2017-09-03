#! /usr/bin/env python

## @file extract_odom_from_bagfile.py
#
# @author Saif Sidhik (sxs1412@student.bham.ac.uk)
# 
# @project graph_slam_rospkg
# @version 1.0

# Description: Reads Odometry msg sent via topic `/odom' from bagfile, and writes to txt file in the format required by groundtruth_compare.py.
# Usage: 1) rosrun graph_slam extract_odom_from_bagfile.py <path_to_file/file_name> 2) Run .bag file when prompted

import cv2 
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np



bridge = CvBridge()
class DatasetCreator:
    i = 0

    def main(self):
      topic = "/odom"
      rospy.Subscriber("/odom",Odometry,self.writeToFile)
      print 'Ready to begin.. Run bag file.. topic:', topic
      rospy.spin()

    def writeToFile(self,odom):
        try:
            print "writing line", self.i+1
            self.file.write(str(str(odom.header.stamp.secs) + " " + str(odom.header.stamp.nsecs) + " " + str(odom.header.seq) + " " + str(odom.pose.pose.position.x) +" "+ str(odom.pose.pose.position.y) + " " + str(odom.pose.pose.position.z) + " " + str(odom.pose.pose.orientation.x) + " " + str(odom.pose.pose.orientation.y) + " " + str(odom.pose.pose.orientation.z) + " " + str(odom.pose.pose.orientation.w)+"\n"))
            self.i += 1
        except KeyboardInterrupt:
            rospy.Unsubscribe()  
            self.file.close()  

if __name__ == '__main__':
    print "Initialising... Please wait..."
    dc = DatasetCreator()
    rospy.init_node('ardrone_odom', anonymous=True)
    if len(sys.argv) > 1:
        filename = str(sys.argv[1])
        if filename[-4:] != ".txt":
            filename += ".txt"
    else:
        filename = "testfile.txt"
    dc.file = open(filename,"w")
    print "File Name:", filename
    dc.main()
    print "Done. Total lines written: ", dc.i+1
