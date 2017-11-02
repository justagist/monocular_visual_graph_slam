# Visual Graph-Based SLAM (ROS Package)

An implementation of Graph-based SLAM using just a sequence of image from a monocular camera. It performs feature-based visual odometry (requires STAM library) and graph optimisation using g2o library.

#### Dependencies

* OpenCV 2.4.x (non-free version)
* [LAPACK](http://www.netlib.org/lapack/)
* g2o (included. Follow installation instructions)
* [Suite-Sparse](https://launchpad.net/ubuntu/+source/suitesparse/)

### TODO
* Remove dependency on PCL (not presently using the library any more)

### Installation (Tested on ROS indigo + Ubuntu 14.04)
    cd <catkin_workspace>/src/
    git clone https://justagist@bitbucket.org/justagist/graph_slam.git
    
    ## build g2o library ##
    cd graph_slam/EXTERNAL/g2o/ 
    cmake . && make 
    
    ## clone STAM library (branch - graphslam_mod) for visual odometry ##
    cd ..
    git clone -b graphslam_mod https://github.com/eaa3/STAM.git
    cd STAM/
    mkdir build && cd build
    cmake ..
    make -j
    
    ## build package ##
    cd <catkin_workspace>/
    catkin_make
    source devel/setup.bash
    
### Usage

Before running the graph-slam node, the location of the 'fabmap_training_data' folder has to be entered by editing the value of '*fabmap_training_data_destination_*' parameter in graph_slam/src/main.cpp file.
The path to the dataset on which the algorithm is to be run can also be set in the main.cpp file.

##### Running graph-slam node:

    rosrun graph_slam main_slam_node <scene_number> [visualize?] [publish rostopics?] [save trajectory to txt file?] [run graph optimisation thread?] [baseline for visual odometry] [trajectory file name] 
Run `rosrun graph_slam main_slam_node` for detailed usage instructions

##### Other nodes
* Plot trajectory output from graph_slam: `rosrun graph_slam trajectory_plotter.py <trajectory_file.txt>` 
* Plot and compare 2 trajectories: `rosrun graph_slam compare_trajectories.py <trajectory_file_1.txt> <trajectory_file_2.txt>`
* Compare trajectory with ground_truth (if available): `rosrun graph_slam groundtruth_compare.py <trajectory_file.txt> <groundtruth.txt>`  

**Function and usage of all nodes are described in the respective source files, along with the format of the input files (where required).**
