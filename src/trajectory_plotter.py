#! /usr/bin/env python
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# USAGE: rosrun graph_slam trajectory plotter <trajectory_file.txt>

def main(argv):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    with open(str(argv[0])) as f:
        lines = f.readlines()
        x = [float(line.split()[1]) for line in lines]
        y = [float(line.split()[2]) for line in lines]
        z = [float(line.split()[3]) for line in lines]
        # print x
    ax.plot(x, y, z,label = 'curve')
    ax.scatter(x[0],y[0],z[0],c='g',marker='x',s=500)
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
    if len(sys.argv) < 2:
        sys.exit("USAGE: rosrun graph_slam trajectory_plotter.py <trajectory_file.txt>")
    main(sys.argv[1:])

