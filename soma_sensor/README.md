# soma_sensor

## __Overview__
SOMAに搭載されているセンサを扱うためのパッケージ

## __Requirements__
* velodyne : velodyne社LiDAR(VLP-16)を扱うパッケージ  
* realsense-ros : RGB-Dカメラ(Realsense, R200/D435/D435iを扱うパッケージ  
* xsens_driver : xsensのIMU(MTi-30)を扱うパッケージ  
* nmea_navsat_driver : USB接続のGPSを扱うパッケージ

## __How to install required packages__
パッケージをインストールする方法は主に二つ  

1. aptを使ってROSの公式パッケージをインストールする  
2. ワークスペースにソースをダウンロードしてビルドする  

多くの場合,1の方法で済む．世界の開発者たちがROS用のパッケージとして開発していて，かつaptのリポジトリに登録されていない場合は2の方法で公開されているソースコードを落としてビルドしなければならない．

### velodyne  (http://wiki.ros.org/velodyne)  
```
sudo apt install ros-melodic-velodyne
```
### realsense-ros (https://github.com/IntelRealSense/realsense-ros#installation-instructions)
```
sudo apt install ros-melodic-realsense2-camera
```
### xsens_driver (http://wiki.ros.org/xsens_driver) 
catkin_wsにソースコードをダウンロードしてビルドする必要がある 
```
cd catkin_ws/src
git clone https://github.com/ethz-asl/ethzasl_xsens_driver
cd ~/catkin_ws
catkin_make
```
### nmea_navsat_driver (http://wiki.ros.org/nmea_navsat_driver) 
```
sudo apt install ros-melodic-nmea-navsat-driver
```

## __How to run sensors__
LiDAR (VLP-16)
```
roslaunch soma_sensor lidar.launch
```
RGB-Dカメラ
* 単体起動  
シリアル番号の指定なし
```
roslaunch soma_sensor rgbd_camera.launch 
```
* 前方・後方カメラ起動  
launchファイル内でシリアル番号指定
```
roslaunch soma_sensor rgbd_camera_F.launch
roslaunch soma_sensor rgbd_camera_B.launch
```
IMU (姿勢計測)
```
roslaunch soma_sensor imu.launch
```
GPS受信 (USB serial communication)
```
roslaunch soma_sensor gps.launch
```