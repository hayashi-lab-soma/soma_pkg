# Metapackage **soma_pkg**

林研究室 林業用ロボット"SOMA"の ROS ベースパッケージ

![SOMA 3D Model](./images/ATV_3D_Model.png "SOMA")

<<<<<<< HEAD

## パッケージ構成 (Structure)
__soma_ros_pkg__  
- __soma_description__ : SOMA's description files (URDF and so on).
- 


## 外部パッケージ要求(Requirement)  
* costmap_2d : 二次元コストマップを扱うパッケージ
* dwa_local_planner : DWAによる局所動作計画を行なうためのパッケージ
* navfn : ナビゲーション用の基礎パッケージ
* jsk_rviz_plugins : rvizに対応した便利なGUIプラグイン集
=======
## Requirements / 外部パッケージ要求
>>>>>>> soma_simulation

VISUALIZATION

- **jsk_rviz_plugins** : rviz に対応した便利な GUI プラグイン集

<<<<<<< HEAD
* __costmap_2d__  
2次元コストマップ用パッケージ
=======
>>>>>>> soma_simulation
```
sudo apt install ros-melodic-jsk-rviz-plugins
```

<<<<<<< HEAD
* __dwa_local_planner__  
DWA 局所動作計画用パッケージ
=======
MOTORS

- _maxon_epos_ros_ :

```
cd catkin_ws/src
git clone https://github.com/iwata-lab/maxon_epos_ros.git
cd ~/catkin_ws
catkin_make
```

SENSORS

- GPS

  - **geodesy** :

    ```
    sudo apt-get install ros-melodic-geodesy
    ```

  - **nmea_msgs** :

    ```
    sudo apt-get install ros-melodic-nmea-msgs
    ```

- RGB-D camera

  - **realsense2_camera** :

    ```
    sudo apt-get install ros-melodic-realsense2-camera
    ```

  - **realsense2_description** :

    ```
    sudo apt-get install ros-melodic-realsense2-description
    ```

  - _realsense_ :

    ```
    cd catkin_ws/src
    git clone https://github.com/pal-robotics-forks/realsense.git
    cd ~/catkin_ws
    catkin_make
    ```

  - _realsense_gazebo_plugin_ :

    ```
    cd catkin_ws/src
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
    cd ~/catkin_ws
    catkin_make
    ```

- LIDAR

  - **velodyne** :

    ```
    sudo apt-get install ros-melodic-velodyne
    ```

  - **pointcloud_to_laserscan** :

    ```
    sudo apt-get install ros-melodic-pointcloud-to-laserscan
    ```

DRIVING

- **ackermann_steering_controller** :

>>>>>>> soma_simulation
```
sudo apt-get install ros-melodic-ackermann-steering-controller
```

<<<<<<< HEAD
* __navfn__  
for melodic
=======
- _steer_drive_ros_ :

```
cd catkin_ws/src
git clone https://github.com/CIR-KIT/steer_drive_ros.git
cd ~/catkin_ws
catkin_make
```

IMAGE PROCESSING

- **pcl_ros** :

>>>>>>> soma_simulation
```
sudo apt-get install ros-melodic-pcl-ros
```

<<<<<<< HEAD
* __jsk_rviz_plugins__  
rviz用の便利なGUIプラグイン集
=======
- _ndt_omp_ :

>>>>>>> soma_simulation
```
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
cd ~/catkin_ws
catkin_make
```

<<<<<<< HEAD
* __rtabmap_ros__
=======
- _fast_gicp_ :

>>>>>>> soma_simulation
```
cd catkin_ws/src
git clone https://github.com/SMRT-AIST/fast_gicp.git
cd ~/catkin_ws
catkin_make
```

<<<<<<< HEAD
* __hdl_graph_slam__  
https://github.com/koide3/hdl_graph_slam  
=======
LOCALIZATION & SLAM

- **move_base**

  - **costmap_2d** : 二次元コストマップを扱うパッケージ

    ```
    sudo apt install ros-melodic-costmap-2d
    ```
>>>>>>> soma_simulation

  - **navfn** : ナビゲーション用の基礎パッケージ

    ```
    sudo apt install ros-melodic-navfn
    ```

  - **dwa_local_planner** : DWA による局所動作計画を行なうためのパッケージ

    ```
    sudo apt install ros-melodic-dwa-local-planner
    ```

- **libg2o** :

<<<<<<< HEAD
=======
```
sudo apt-get install ros-melodic-libg2o
```

- **rtabmap_ros** :

>>>>>>> soma_simulation
```
sudo apt install ros-melodic-rtabmap-ros
```

- _hdl_graph_slam_ : LiDAR に対応した３次元 slam パッケージ

```
cd catkin_ws/src
git clone https://github.com/koide3/hdl_graph_slam.git
cd ~/catkin_ws
catkin_make
```

- _hdl_global_localization_ :

```
cd catkin_ws/src
git clone https://github.com/koide3/hdl_global_localization.git
cd ~/catkin_ws
catkin_make
```

## Structure / パッケージ構成

- **minisoma** : ?

- **soma_atv_driver** : Motor control

- **soma_base** : Navigation with **move_base**

- **soma_bring_up** : Sensors control / センサ関係の launch,node を持つパッケージ

- **soma_description** : Robot model, visualization in Rviz, simulation in Gazebo, driving with rqt_robot_steering GUI

- **soma_experiments** : ?

- **soma_global_planner** : Why not in **soma_base** ? / 大域的な動作計画のためのパッケージ

- **soma_localization** : Localization in known map

- **soma_mapping** : Mapping of environment

- **soma_mse** : ?

- **soma_msgs** : Custom ROS messages

- **soma_online_slam** : Online Simultaneous Localization And Mapping (SLAM)

- **soma_perception** : Slope detection / 点群処理，画像処理系を持つパッケージ

- **soma_samples** : ? / 動作チェック用，お試し用の置き場所

- **soma_tools** : ?

<<<<<<< HEAD
=======
- **soma_viz** : ?
>>>>>>> soma_simulation
