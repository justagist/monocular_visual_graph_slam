#! /usr/bin/env python

## @file groundtruth_compare.py
#
# @author Saif Sidhik (sxs1412@student.bham.ac.uk)
# 
# @project graph_slam_rospkg
# @version 1.0

# Description: Draws 2 plots in the same window. Used to compare output of graph_slam with the ground truth.
# USAGE: rosrun graph_slam groundtruth_compare <trajectory_file_1.txt> <ground_truth_file.txt>
# Inputs: (1) output txt file from running main_slam_node (2) output file from running extract_odom_from_bagfile.py for the corresponding dataset.

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def main(arguments):
    fig = plt.figure()
    fig.suptitle('Compare With GroundTruth')
    ax = fig.add_subplot(1,1,1, projection='3d')
    plot1= []

    # Estimated Trajectory --
    with open(str(arguments[0])) as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].split()[0] != 'SLAM':
                plot1.append(lines[i])
            else:
                for line in range(i,len(lines)):
                    print lines[line]
                break

        x = [float(point.split()[1]) for point in plot1]
        y = [float(point.split()[2]) for point in plot1]
        z = [float(point.split()[3]) for point in plot1]

    ax.plot(x, y, z,label = 'estimated path')
    ax.scatter(x[0],y[0],z[0],c='g',marker='x',s=500)
    ax.scatter(x[len(x)-1],y[len(y)-1],z[len(z)-1],c='c',marker='x',s=300)


    # Ground Truth Plot --
    with open(str(arguments[1])) as g:
        glines = g.readlines()
        x1 = [1000*float(line.split()[3]) for line in glines]
        y1 = [1000*float(line.split()[4]) for line in glines]
        z1 = [1000*float(line.split()[5]) for line in glines]
        
    ax.plot(x1,y1,z1, label= 'ground_truth')
    ax.scatter(x1[0],y1[0],z1[0],c='r',marker='x',s=500)
    ax.legend()

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit("USAGE: rosrun graph_slam groundtruth_compare.py <trajectory_file.txt> <groundtruth_file.txt>")
    main(sys.argv[1:])

