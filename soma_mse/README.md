# Package **soma_mse**

# はじめに

本パッケージは，MSE との共同研究開発で進めた自動伐倒機のソフトウェアおよび，launch ファイル，yaml ファイルまとめている．

# システム構成(System Structure)

## デバイス構成(Device connection diagram)

## 動作シーケンス(Behavior seaquence)

# 実行方法（How to use）

## 1. **センサノード起動**

LiDAR(velodyne, VLP-16)，IMU(Xsense, MTi-30)，RTK-GPS(emlid,)をまとめて起動する．

### **起動方法**

```
roslaunch soma_mse sensor.launch
```

### **引数**

()内はデフォルト値．

- **base_frame_id** (base_link) : ロボット本体の frame 名
- **gui** (true) : rviz 起動フラグ

#### rviz 付きの起動方法

```
roslaunch soma_mse sensor.launch gui:=true
```

## 2. **SLAM ノード起動**

LiDAR ベース SLAM である[HDL Graph SLAM](https://github.com/koide3/hdl_graph_slam) を使う．
[HDL Graph SLAM](https://github.com/koide3/hdl_graph_slam)を参照して，パッケージのインストールが必要．

### **基本の起動方法**

```
roslaunch soma_mse hdl_graph_slam.launch
```

### **引数**

()内はデフォルト値．

- **base_frame_id** (base_link) : ロボット本体の frame 名
- **lidar_link** (velodyen) : LiDAR の frame 名
- **enable_manager** (false) : velodyne_nodelet_manager の起動フラグ．velodyne のセンサノードが動いている場合は false，gazebo シミュレータ・bag ファイルから velodyne_points をサブスクライブする場合等は true にすること．
- **gui** (false) : rviz 起動フラグ
- **distance_near_thresh** (0.1) : SLAM に用いる点群の最低距離閾値 (m)
- **distance_far_thresh** (20.0) : SLAM に用いる点群の最高距離閾値 (m)

### velodyne_nodelet_manager なし，rviz 付きの起動方法

```
roslaunch soma_mse hdl_graph_slam enable_manager:=false gui:=true
```

### velodyne_nodelet_manager 有り，rviz 付きの起動方法

```
roslaunch soma_mse hdl_graph_slam enable_manager:=true gui:=true
```

## 3. **Harvesting Behavior ノード起動**

MSE 用，自動伐倒作業用の自律移動ノード．  
ソースファイル [behavior.cpp](./src/node/behavior_node.cpp)

### **基本の起動方法**

```
roslaunch soma_mse harvesting_behavior.launch
```

### **引数**

()内はデフォルト値．

- **base_frame_id** (base_link) : ロボット本体の frame 名
- **odom_frame_id** (base_link) : オドメトリ座標系の frame 名
- **map_frame_id** (base_link) : マップ座標系の frame 名
- **gui** (true) : rviz 起動フラグ．絶対必要なのでデフォルトで true
