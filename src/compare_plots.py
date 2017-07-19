#! /usr/bin/env python
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

def main(arguments):
    fig = plt.figure()
    fig.suptitle('A tale of 2 subplots')
    ax = fig.add_subplot(1,1,1, projection='3d')
    with open(str(arguments[0])) as f:
        lines = f.readlines()
        x = [float(line.split()[1]) for line in lines]
        y = [float(line.split()[2]) for line in lines]
        z = [float(line.split()[3]) for line in lines]
    # print x
    ax.plot(x, y, z,label = 'estimated path')
    ax.scatter(x[0],y[0],z[0],c='g',marker='x',s=500)
    with open(str(arguments[1])) as g:
        glines = g.readlines()
        x1 = [1000*float(line.split()[3]) for line in glines]
        y1 = [1000*float(line.split()[4]) for line in glines]
        z1 = [1000*float(line.split()[5]) for line in glines]
    # print x1
    # ax = fig.add_subplot(2,1,2, projection='3d')
    ax.plot(x1,y1,z1, label= 'ground_truth')
    ax.scatter(x1[0],y1[0],z1[0],c='r',marker='x',s=500)
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
   main(sys.argv[1:])

