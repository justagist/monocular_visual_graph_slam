#! /usr/bin/env python
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# USAGE: rosrun graph_slam compare_trajectories.py <trajectory_file_1.txt> <trajectory_file_2.txt>

def main(arguments):
    fig = plt.figure()
    fig.suptitle('Compare Plots')
    ax = fig.add_subplot(1,1,1, projection='3d')
    with open(str(arguments[0])) as f:
        lines = f.readlines()

        if lines[len(lines)-1].split()[0] == '***':
            print "trajectory 1:", lines[len(lines)-1]
            lines = lines[:-1]

        # x = [float(line.split()[1]) for line in lines]
        # y = [float(line.split()[2]) for line in lines]
        # z = [float(line.split()[3]) for line in lines]
    # # print x
    # ax.plot(x, y, z,label = 'trajectory 1')
    # ax.scatter(x[0],y[0],z[0],c='g',marker='x',s=500)
    with open(str(arguments[1])) as g:
        glines = g.readlines()

        if glines[len(glines)-1].split()[0] == 'info::':
            print "trajectory 2:", glines[len(lines)-1]
            glines = glines[:-1]

    #     x1 = [float(line.split()[1]) for line in glines]
    #     y1 = [float(line.split()[2]) for line in glines]
    #     z1 = [float(line.split()[3]) for line in glines]
    # # print x1
    # # ax = fig.add_subplot(2,1,2, projection='3d')
    # ax.plot(x1,y1,z1, label= 'trajectory 2')
    # ax.scatter(x1[0],y1[0],z1[0],c='r',marker='x',s=500)
    # ax.scatter(x1[len(x1)-1],y1[len(y1)-1],z1[len(z1)-1],c='r',marker='x',s=300)
    # ax.legend()

    
    # ax.set_xlabel('z')
    # ax.set_ylabel('x')
    # ax.set_zlabel('y')
    # # ax.set_xlim(-1000, 1000)
    # # ax.set_ylim(-200, 2000)
    # # ax.set_zlim(-1000, 0)
    # ax.invert_zaxis()
    # ax.invert_xaxis()


    # plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit("USAGE: rosrun graph_slam compare_trajectories.py <trajectory_file_1.txt> <trajectory_file_2.txt>")
    main(sys.argv[1:])

