# <p style="text-align: center;">soma_pkg</p>

林研究室 林業用ロボット"SOMA"の ROS ベースパッケージ

![SOMA 3D Model](./images/ATV_3D_Model.png "SOMA")

## Requirements / 外部パッケージ要求

VISUALIZATION

- jsk_rviz_plugins : rviz に対応した便利な GUI プラグイン集

```
sudo apt install ros-melodic-jsk-rviz-plugins
```

MOTORS

- maxon_epos_ros :

```
cd catkin_ws/src
git clone https://github.com/iwata-lab/maxon_epos_ros.git
cd ~/catkin_ws
catkin_make
```

SENSORS

- GPS

  - geodesy :

    ```
    sudo apt-get install ros-melodic-geodesy
    ```

  - nmea_msgs :

    ```
    sudo apt-get install ros-melodic-nmea-msgs
    ```

- RGB-D camera

  - realsense2_camera :

    ```
    sudo apt-get install ros-melodic-realsense2-camera
    ```

  - realsense2_description :

    ```
    sudo apt-get install ros-melodic-realsense2-description
    ```

  - realsense :

    ```
    cd catkin_ws/src
    git clone https://github.com/pal-robotics-forks/realsense.git
    cd ~/catkin_ws
    catkin_make
    ```

  - realsense_gazebo_plugin :

    ```
    cd catkin_ws/src
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
    cd ~/catkin_ws
    catkin_make
    ```

- LIDAR

  - velodyne :

    ```
    sudo apt-get install ros-melodic-velodyne
    ```

  - pointcloud_to_laserscan :

    ```
    sudo apt-get install ros-melodic-pointcloud-to-laserscan
    ```

DRIVING

- ackermann_steering_controller :

```
sudo apt-get install ros-melodic-ackermann-steering-controller
```

- steer_drive_ros :

```
cd catkin_ws/src
git clone https://github.com/CIR-KIT/steer_drive_ros.git
cd ~/catkin_ws
catkin_make
```

IMAGE PROCESSING

- pcl_ros :

```
sudo apt-get install ros-melodic-pcl-ros
```

- ndt_omp :

```
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
cd ~/catkin_ws
catkin_make
```

- fast_gicp :

```
cd catkin_ws/src
git clone https://github.com/SMRT-AIST/fast_gicp.git
cd ~/catkin_ws
catkin_make
```

LOCALIZATION & SLAM

- move_base

  - costmap_2d : 二次元コストマップを扱うパッケージ

    ```
    sudo apt install ros-melodic-costmap-2d
    ```

  - navfn : ナビゲーション用の基礎パッケージ

    ```
    sudo apt install ros-melodic-navfn
    ```

  - dwa_local_planner : DWA による局所動作計画を行なうためのパッケージ

    ```
    sudo apt install ros-melodic-dwa-local-planner
    ```

- libg2o :

```
sudo apt-get install ros-melodic-libg2o
```

- rtabmap_ros :

```
sudo apt install ros-melodic-rtabmap-ros
```

- hdl_graph_slam : LiDAR に対応した３次元 slam パッケージ

```
cd catkin_ws/src
git clone https://github.com/koide3/hdl_graph_slam.git
cd ~/catkin_ws
catkin_make
```

- hdl_global_localization :

```
cd catkin_ws/src
git clone https://github.com/koide3/hdl_global_localization.git
cd ~/catkin_ws
catkin_make
```

## Structure / パッケージ構成

- soma_atv_driver

- soma_base

- soma_bring_up : センサ関係の launch,node を持つパッケージ

- soma_description

- soma_experiments

- soma_global_planner : 大域的な動作計画のためのパッケージ

- soma_localization

- soma_mapping

- soma_mse

- soma_msgs

- soma_perception : 点群処理，画像処理系を持つパッケージ

- soma_samples : 動作チェック用，お試し用の置き場所

- soma_tools

- soma_viz
