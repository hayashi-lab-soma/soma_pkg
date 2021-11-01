# Package **soma_atv_driver**  

KFX®90の走行制御パッケージ．MAXONモータおよびEPOSの制御を行い，走行中の状態と速度制御を行うノードが含まれている．また，前輪側ロータリエンコーダのパルスカウントおよび速度計測値取得と，電磁クラッチ制御も行う．

## **Requirements**  
- [MAXON EPOS Command Library][1]  
EPOSとのUSBシリアル通信を行うためのライブラリをインストールする必要がある．
詳細はリンク先の"EPOS Command Library"よりドキュメンテーションに記載されている．
  
- [maxon_epos_ros Package][2]  
    iwata-labさんが配布しているEPOS通信用のROSパッケージ．メッセージも作ってくれているのでつかわせてもらう．
    ```
    $ cd catkin_ws/src
    $ git clone https://github.com/iwata-lab/maxon_epos_ros.git
    $ cd ~/catkin_ws
    $ catkin_make
    ```

- [joy Package][3] and [teleop-twist-joy Package][4]
  ```
  $ sudo apt install ros-melodic-joy
  $ sudo apt install ros-melodic-teleop-twist-joy
  ```

## **Nodes or Nodelets**
- `atv_driver_node`
- `maxon_bringup` (External, maxon_epos_ros package)
- `joy_node` (External, joy package)
- `teleop_node` (External, teleop-twist-joy package)

<img src=./images/nodes_structure.png width=80%>

## **Usage**  
### Remote control by joy stick controller
```
$ roslaunch soma_atv_driver maxon_bringup.launch
$ roslaunch soma_atv_driver atv_driver.launch
$ roslaunch soma_atv_driver joy_teleop.launch
```

## **Parameters**  
### joy stick parameters  
See `soma_atv_driver/launch/joy_teleop.launch`  
```
<param name="scale_linear" value="1.0" />  
<param name="scale_angular" value="0.52" />
```

### motor parameters  
See `soma_atv_driver/config/motors.yaml`  
Please confirm loading config file in `soma_atv_driver/launch/maxon_bringup.launch`
```
<node name="maxon_bringup" pkg="maxon_epos_driver" type="maxon_bringup" output="screen">
    <rosparam command="load" file="$(find soma_atv_driver)/config/motors.yaml" />
</node>
```

### atv driver parameters
**coming soon ...**
```
```



[1]:https://maxonjapan.com/mmc/#command_library
[2]:https://github.com/iwata-lab/maxon_epos_ros.git
[3]:http://wiki.ros.org/joy
[4]:http://wiki.ros.org/teleop_twist_joy