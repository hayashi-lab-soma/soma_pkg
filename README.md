# Metapackage **soma_pkg**

林研究室 林業用ロボット"SOMA"の ROS ベースパッケージ

![SOMA 3D Model](./images/ATV_3D_Model.png "SOMA")

## Requirements / 外部パッケージ要求

VISUALIZATION

- **jsk_rviz_plugins** : rviz に対応した便利な GUI プラグイン集

```
sudo apt install ros-melodic-jsk-rviz-plugins
```

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

```
sudo apt-get install ros-melodic-ackermann-steering-controller
```

- _steer_drive_ros_ :

```
cd catkin_ws/src
git clone https://github.com/CIR-KIT/steer_drive_ros.git
cd ~/catkin_ws
catkin_make
```

IMAGE PROCESSING

- **pcl_ros** :

```
sudo apt-get install ros-melodic-pcl-ros
```

- _ndt_omp_ :

```
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
cd ~/catkin_ws
catkin_make
```

- _fast_gicp_ :

```
cd catkin_ws/src
git clone https://github.com/SMRT-AIST/fast_gicp.git
cd ~/catkin_ws
catkin_make
```

LOCALIZATION & SLAM

- **move_base**

  - **costmap_2d** : 二次元コストマップを扱うパッケージ

    ```
    sudo apt install ros-melodic-costmap-2d
    ```

  - **navfn** : ナビゲーション用の基礎パッケージ

    ```
    sudo apt install ros-melodic-navfn
    ```

  - **dwa_local_planner** : DWA による局所動作計画を行なうためのパッケージ

    ```
    sudo apt install ros-melodic-dwa-local-planner
    ```

- **libg2o** :

```
sudo apt-get install ros-melodic-libg2o
```

- **rtabmap_ros** :

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

- **soma_perception** : Slope detection / 点群処理，画像処理系を持つパッケージ

- **soma_samples** : ? / 動作チェック用，お試し用の置き場所

- **soma_tools** : ?

- **soma_viz** : ?
