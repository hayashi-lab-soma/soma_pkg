# **soma_pkg**  
林研究室林業用ロボット"SOMA"の ROS パッケージ  
<img src=./images/ATV_3D_Model.png width=60%>

## **REQUIREMENTS** / 外部パッケージ要求  
## **VISUALIZATION**  
- `jsk_rviz_plugins`  
```
$ sudo apt install ros-melodic-jsk-rviz-plugins
```
### **MOTORS**  
- `maxon_epos_ros`  
```
$ cd catkin_ws/src
$ git clone https://github.com/iwata-lab/maxon_epos_ros.git
$ cd ~/catkin_ws
$ catkin_make
```

### **SENSORS**  
- GPS  
  - `geodesy`  
    ```
    $ sudo apt install ros-melodic-geodesy
    ```  
  - `nmea_msgs`  
    ```
    $ sudo apt install ros-melodic-nmea-msgs
    ```

- RGB-D camera
  - `realsense2_camera`  
    ```
    $ sudo apt-get install ros-melodic-realsense2-camera
    ```
  - `realsense2_description`  
    ```
    $ sudo apt install ros-melodic-realsense2-description
    ```
  - `realsense`  
    ```
    $ cd catkin_ws/src
    $ git clone https://github.com/pal-robotics-forks/realsense.git
    $ cd ~/catkin_ws
    $ catkin_make
    ```
  - `realsense_gazebo_plugin`  
    ```
    $ cd catkin_ws/src
    $ git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
    $ cd ~/catkin_ws
    $ catkin_make
    ```

- LIDAR  
  - `velodyne`  
    ```
    $ sudo apt-get install ros-melodic-velodyne
    ```
  - `pointcloud_to_laserscan`  
    ```
    $ sudo apt-get install ros-melodic-pointcloud-to-laserscan
    ```

### **DRIVING**  
- `ackermann_steering_controller`  
```
$ sudo apt-get install ros-melodic-ackermann-steering-controller
```

- `steer_drive_ros`
```
$ cd catkin_ws/src
$ git clone https://github.com/CIR-KIT/steer_drive_ros.git
$ cd ~/catkin_ws
$ catkin_make
```

### **IMAGE PROCESSING**  
- `pcl_ros`  
```
$ sudo apt-get install ros-melodic-pcl-ros
```

- `ndt_omp`  
```
$ cd catkin_ws/src
$ git clone https://github.com/koide3/ndt_omp.git
$ cd ~/catkin_ws
$ catkin_make
```

- `fast_gic`  
```
$ cd catkin_ws/src
$ git clone https://github.com/SMRT-AIST/fast_gicp.git
$ cd ~/catkin_ws
$ catkin_make
```

### **LOCALIZATION & SLAM**  
- **move_base**  
  - `costmap_2d`  
  ```
  $ sudo apt install ros-melodic-costmap-2d
  ```

  - `navfn`
  ```
  $ sudo apt install ros-melodic-navfn
  ```

  - `dwa_local_planner`
  ```
  $ sudo apt install ros-melodic-dwa-local-planner
  ```

- `libg2o` :
```
$ sudo apt-get install ros-melodic-libg2o
```

- `rtabmap_ros`  
```
$ sudo apt install ros-melodic-rtabmap-ros
```

- `hdl_graph_slam`
```
$ cd catkin_ws/src
$ git clone https://github.com/koide3/hdl_graph_slam.git
$ cd ~/catkin_ws
$ catkin_make
```

- `hdl_global_localization`  
```
$ cd catkin_ws/src
$ git clone https://github.com/koide3/hdl_global_localization.git
$ cd ~/catkin_ws
$ catkin_make
```

## **Structure** / パッケージ構成

- **soma_atv_driver** : Motor control

- **soma_base** : Navigation with **move_base**

- **soma_bring_up** : Sensors control / センサ関係の launch,node を持つパッケージ

- **soma_description** : Robot model, visualization in Rviz, simulation in Gazebo, driving with rqt_robot_steering GUI

- **soma_experiments** : ?

- **soma_global_planner** : Why not in **soma_base** ? / 大域的な動作計画のためのパッケージ

- **soma_localization** : Localization in known map

- **soma_mapping** : Mapping of environment

- **soma_msgs** : Custom ROS messages

- **soma_online_slam** : Online Simultaneous Localization And Mapping (SLAM)

- **soma_perception** : Slope detection / 点群処理，画像処理系を持つパッケージ

- **soma_samples** : ? / 動作チェック用，お試し用の置き場所

- **soma_tools** : ?

- **soma_viz** : ?
