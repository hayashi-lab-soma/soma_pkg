# soma_pkg
林研究室 林業用ロボット"SOMA"のROSベースパッケージ

![SOMA 3D Model](./images/ATV_3D_Model.png "SOMA")


## パッケージ構成 (Structure)
soma_ros_pkg  
- 
- 


## 外部パッケージ要求(Requirement)  
* costmap_2d : 二次元コストマップを扱うパッケージ
* dwa_local_planner : DWAによる局所動作計画を行なうためのパッケージ
* navfn : ナビゲーション用の基礎パッケージ
* jsk_rviz_plugins : rvizに対応した便利なGUIプラグイン集

* hdl_graph_slam : LiDARに対応した３次元slamパッケージ  

* __costmap_2d__  
2次元コストマップ用パッケージ
```
sudo apt install ros-melodic-costmap-2d
```

* __dwa_local_planner__  
DWA 局所動作計画用パッケージ
```
sudo apt install ros-melodic-dwa-local-planner
```

* __navfn__  
for melodic
```
sudo apt install ros-melodic-navfn
```

* __jsk_rviz_plugins__  
rviz用の便利なGUIプラグイン集
```
sudo apt install ros-melodic-jsk-rviz-plugins
```

* __rtabmap_ros__
```
sudo apt install ros-melodic-rtabmap-ros
```

* __hdl_graph_slam__  
https://github.com/koide3/hdl_graph_slam  

catkin_wsにソースコードをダウンロードしてcatkin_makeする  
HDL Graph SLAMに必要なパッケージもインストールする必要がある  

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


