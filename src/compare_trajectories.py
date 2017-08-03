#! /usr/bin/env python
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# USAGE: rosrun graph_slam compare_trajectories.py <trajectory_file_1.txt> <trajectory_file_2.txt>

def main(arguments):
    fig = plt.figure()
    fig.suptitle('Compare Plots')
    ax = fig.add_subplot(1,1,1, projection='3d')
    plot1 = []
    print "\nTRAJECTORY 1:\n"
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

    f.close()


    ax.plot(x, y, z,label = 'trajectory 1')
    ax.scatter(x[0],y[0],z[0],c='g',marker='x',s=500)
    plot2 = []
    print "\nTRAJECTORY 2:\n"
    with open(str(arguments[1])) as g:
        lines2 = g.readlines()
        for i in range(len(lines2)):
            if lines2[i].split()[0] != 'SLAM':
                plot2.append(lines2[i])
            else:
                for line in range(i,len(lines2)):
                    print lines2[line]
                break

        x1 = [float(point.split()[1]) for point in plot2]
        y1 = [float(point.split()[2]) for point in plot2]
        z1 = [float(point.split()[3]) for point in plot2]

    g.close()
    
    ax.plot(x1,y1,z1, label= 'trajectory 2')
    ax.scatter(x1[0],y1[0],z1[0],c='r',marker='x',s=500)
    ax.scatter(x1[len(x1)-1],y1[len(y1)-1],z1[len(z1)-1],c='r',marker='x',s=300)
    ax.legend()

    
    # ax.set_xlabel('z')
    # ax.set_ylabel('x')
    # ax.set_zlabel('y')
    # # ax.set_xlim(-1000, 1000)
    # # ax.set_ylim(-200, 2000)
    # # ax.set_zlim(-1000, 0)
    # ax.invert_zaxis()
    # ax.invert_xaxis()


    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit("USAGE: rosrun graph_slam compare_trajectories.py <trajectory_file_1.txt> <trajectory_file_2.txt>")
    main(sys.argv[1:])

