#! /usr/bin/env python

## @file groundtruth_compare.py
#
# @author Saif Sidhik (sxs1412@student.bham.ac.uk)
# 
# @project graph_slam_rospkg
# @version 1.0

# Description: Plots trajectory.
# USAGE: rosrun graph_slam trajectory plotter <trajectory_file.txt>
# Input: output txt file from running main_slam_node 

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def main(arguments):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plot1= []

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
        # print x
    ax.plot(x, y, z,label = 'curve')
    ax.scatter(x[0],y[0],z[0],c='g',marker='x',s=500)
    ax.legend()

    ## ----- for ismar dataset (dont use otherwise) (NOT REQUIRED ANYMORE. FIXED COORDINATES IN GRAPH_SLAM)
    # ax.set_xlabel('z')
    # ax.set_ylabel('x')
    # ax.set_zlabel('y')
    # # ax.set_xlim(-1000, 1000)
    # # ax.set_ylim(-200, 2000)
    # # ax.set_zlim(-1000, 0)
    # ax.invert_zaxis()
    # ax.invert_xaxis()
    # ------------------------------------------


    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("USAGE: rosrun graph_slam trajectory_plotter.py <trajectory_file.txt>")
    main(sys.argv[1:])

