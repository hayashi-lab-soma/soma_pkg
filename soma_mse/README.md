# はじめに
本パッケージは，MSEとの共同研究開発で進めた自動伐倒機のソフトウェアおよび，launchファイル，yamlファイルまとめている．

# システム構成(System Structure)
## デバイス構成(Device connection diagram)
## 動作シーケンス(Behavior seaquence)

# 実行方法（How to use）
## 1. __センサノード起動__  
LiDAR(velodyne, VLP-16)，IMU(Xsense, MTi-30)，RTK-GPS(emlid,)をまとめて起動する．  
### **起動方法**  
```
roslaunch soma_mse sensor.launch
```
### **引数**  
()内はデフォルト値． 
  * __base_frame_id__ (base_link) : ロボット本体のframe名
  * __gui__ (true) : rviz起動フラグ

#### rviz付きの起動方法
```
roslaunch soma_mse sensor.launch gui:=true
```

## 2. __SLAMノード起動__
LiDARベースSLAMである[HDL Graph SLAM](https://github.com/koide3/hdl_graph_slam) を使う．
[HDL Graph SLAM](https://github.com/koide3/hdl_graph_slam)を参照して，パッケージのインストールが必要．

### __基本の起動方法__  
```
roslaunch soma_mse hdl_graph_slam.launch
```

### __引数__  
()内はデフォルト値． 
* __base_frame_id__ (base_link) : ロボット本体のframe名
* __lidar_link__ (velodyen) : LiDARのframe名
* __enable_manager__ (false) : velodyne_nodelet_managerの起動フラグ．velodyneのセンサノードが動いている場合はfalse，gazeboシミュレータ・bagファイルからvelodyne_pointsをサブスクライブする場合等はtrueにすること．
* __gui__ (false) : rviz起動フラグ
* __distance_near_thresh__ (0.1) : SLAMに用いる点群の最低距離閾値 (m)
* __distance_far_thresh__ (20.0) : SLAMに用いる点群の最高距離閾値 (m)
### velodyne_nodelet_managerなし，rviz付きの起動方法
```
roslaunch soma_mse hdl_graph_slam enable_manager:=false gui:=true
```
### velodyne_nodelet_manager有り，rviz付きの起動方法
```
roslaunch soma_mse hdl_graph_slam enable_manager:=true gui:=true
```

## 3. __Harvesting Behavior ノード起動__  
MSE用，自動伐倒作業用の自律移動ノード．  
ソースファイル [behavior.cpp](./src/node/behavior_node.cpp)

### __基本の起動方法__ 
```
roslaunch soma_mse harvesting_behavior.launch
```

### __引数__  
()内はデフォルト値．
* __base_frame_id__ (base_link) : ロボット本体のframe名
* __odom_frame_id__ (base_link) : オドメトリ座標系のframe名
* __map_frame_id__ (base_link) : マップ座標系のframe名
* __gui__ (true) : rviz起動フラグ．絶対必要なのでデフォルトでtrue