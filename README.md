# 简介


## 声明
该代码基于easy-handeye，使用棋盘格而并不是aruco，easy-handeye主页参见https://github.com/IFL-CAMP/easy_handeye

该代码已在ubuntu18.04中完成测试。

## 依赖
ros-melodic

opencv $\geqq$ 4

moveit
 
 ## 安装
 
 ```
 mkdir catkin_ws
 mkdir src
 cd src
 git clone https://github.com/nidhc/chessboard_hand_eye
 cd ..
 rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
 catkin_make
 ```
 
 ## 快速使用
 
 以使用12 $\times$ 9棋盘格，方格边长25cm，franka机械臂，realsense相机和眼在手上为例
 ```
 source catkin_ws\devel\setup.bash
 roslaunch easy_handeye panda_realsense_eyeonbase.launch
 ```
 点击check start point确认以当前位置为起始点，算法在起始点附近选择17个不同的位姿。依次点击next plan execute就可以运动到下一个位置。
 
 选择不同的点为起始点可以录制多组数据
 
 每次运动后使用
 ```
 rosrun rqt_image_view rqt_image_view
```
确认相机可以完整的拍到标定板

点击take sample可以记录当前数据，点击compute可以得到标定结果，可视化界面中只有xyz和四元数，终端中可以看到位姿变换矩阵

## 使用说明

如果需要更换相机，机械臂或者标定板需要修改panda_realsense_eyeonbase.launch文件

### 更换相机
修改
```
    <include file="$(find realsense2_camera)/launch/rs_camera.launch"> </include>     
 <!-- 相机的启动文件 -->
<remap from="/camera_info" to="/camera/color/camera_info" />
 <!-- 发布相机参数的话题名 -->
<remap from="/image" to="/camera/color/image_raw" />
 <!-- 发布相机图片的话题名 -->
<param name="camera_frame"       value="camera_color_optical_frame"/>
 <!-- 相机图片对应的坐标系 -->
 <arg name="tracking_base_frame" value="camera_color_optical_frame"/>
 <!-- 相机图片对应的坐标系 -->
```

### 更换机械臂
```
 <include file="$(find panda_moveit_config)/launch/franka_control.launch">
 <!-- 机械臂moveit启动文件-->
 <arg name="robot_ip" value="172.16.0.2"/> <!-- set your robot ip -->
 <!-- 机械臂ip -->
  <arg name="move_group" value="panda_manipulator"  doc="the name of move_group for the automatic robot motion with MoveIt!" />
  <!-- 机械臂运动组 -->
<arg name="robot_base_frame" value="panda_link0"/>
  <!-- 机械臂基坐标系-->
<arg name="robot_effector_frame" value="panda_link8"/>
  <!-- 机械臂末端坐标系-->
```

### 更换标定板
```
<arg name="markerSize"      default="0.025"/>
  <!-- 标定板每个小方块电脑边长，单位m-->
<arg name="markerWidth"     default="8"/>
  <!-- 标定板横向内角点个数-->
<arg name="markerHeight"     default="11"/>
<!-- 标定板纵向内角点个数-->
```

### 更换标定坐标系
标定结果是A坐标系相对于B坐标系的位姿


A坐标系定义
```
<arg name="ref_frame"       default="camera_color_optical_frame"/>
```
B坐标系定义
```
  <arg name="robot_effector_frame" value="panda_link8"/>
```




