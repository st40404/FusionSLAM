# FusionSLAM

Play sensor fusion on tb3 in docker(melodic), you need to build docker first
   - go to docker package
   ```bash
   $ chmod +x build.sh
   $ ./buils.sh
   $ chmod +x run.sh
   $ ./run.sh
   ```

<!-- TOC -->
- [Platform](#Platform)
- [ORBSLAM2](#ORBSLAM2)
- [Setting](#Setting)
- [Execute](#Execute)
<!-- /TOC -->

## Platform
 * System
   - ROS melodic
 * Robot
   - TURTLEBOT3(waffle)
 * Lidar
   - Hokuyo UTM-30LX
 * Camera
   - Intel realsense d435i
  
## ORBSLAM2
 * build dependency (Pangolin)
   - reconstruct /build folder in Pangolin package
   - go to Pangolin/build
   ```bash
   $ cmake ..
   $ make
   $ sudo make install
   ```
 * build ORBSLAM2(we build ROS part in catkin_make)
   - reconstruct /build folder in ORB_SLAM2 package
   - go to ORB_SLAM2/build
   ```bash
   $ chmod +x build.sh
   $ ./build.sh
   ```
 * set param(you need to set parameter)
 * vocabulary in $(find orbslam2_ros)/data/sim_tune.yaml
 * settings in $(find orbslam2_ros)/data/ORBvoc.txt"

## Setting
 * build in ws
   ```bash
   $ catkin_make
   ```

## Execute
```bash
$ roslaunch all_process PLICP_ORB_sim.launch
```


























