# soma_pkg
林研究室 林業用ロボット"SOMA"のROSベースパッケージ

![SOMA 3D Model](./images/ATV_3D_Model.png "SOMA")

## 外部パッケージ要求(Requirement)  
* velodyne : velodyne社製LiDARを扱うパッケージ  
* realsense-ros : R200,D435,D435iを扱うパッケージ  
* xsens_driver : xsensのIMUを扱うパッケージ  
* nmea_navsat_driver : USB接続のGPSを扱うパッケージ  
* costmap_2d : 二次元コストマップを扱うパッケージ
* dwa_local_planner : DWAによる局所動作計画を行なうためのパッケージ
* navfn : ナビゲーション用の基礎パッケージ
* jsk_rviz_plugins : rvizに対応した便利なGUIプラグイン集

* hdl_graph_slam : LiDARに対応した３次元slamパッケージ  








* __costmap_2d__  
```
sudo apt install ros-kinetic-costmap-2d
```
```
sudo apt install ros-melodic-costmap-2d
```

* __dwa_local_planner__  
```
sudo apt install ros-kinetic-dwa-local-planner
```
```
sudo apt install ros-melodic-dwa-local-planner
```

* __navfn__  
for kinetic
```
sudo apt install ros-kinetic-navfn
```
for melodic
```
sudo apt install ros-melodic-navfn
```

* __jsk_rviz_plugins__  
rviz用の便利なGUIプラグイン集
for melodic
```
sudo apt install ros-melodic-jsk-rviz-plugins
```

* __rtabmap_ros__
for melodic
```
sudo apt install ros-melodic-rtabmap-ros
```


* __hdl_graph_slam__  
https://github.com/koide3/hdl_graph_slam  

catkin_wsにソースコードをダウンロードしてcatkin_makeする  
HDL Graph SLAMに必要なパッケージもインストールする必要がある  

for kinetic
```
sudo apt-get install ros-kinetic-geodesy ros-kinetic-pcl-ros ros-kinetic-nmea-msgs ros-kinetic-libg2o
```
for melodic
```
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o
```

```
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/koide3/hdl_graph_slam.git
cd ~/catkin_ws
catkin_make
```


## パッケージ構成(Structure)
soma_ros_pkg:
* soma_motion_planner  
大域的な動作計画のためのパッケージ
* soma_sensor  
センサ関係のlaunch,nodeを持つパッケージ
* soma_perception  
点群処理，画像処理系を持つパッケージ
* soma_ros:
* soma_smpls:  
動作チェック用，お試し用の置き場所

## 実行方法等(How to run)
__センサノード起動__


