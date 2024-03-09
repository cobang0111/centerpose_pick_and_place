# Universal Robot Shoe Organization on ROS2 MoveIt based on Object Pose Estimation (CenterPose) 
## ✨Summary
This repository about Universal Robot's shoe organization based on categorized 3D Object Pose Estimation Algorithm : CenterPose
<br>
<br>
[Youtube Shorts Video](https://youtube.com/shorts/Ih1Q0JDabwE)
<br>
<br>
[Youtube Video](https://youtu.be/F6QBFe8h4Jc)

<br>

![image](https://github.com/cobang0111/centerpose_pick_and_place/assets/97373900/dff004ae-bee8-424b-8563-4c09285225a3)


<br>
Tested on isaac-ros docker container from 
Hardware UR3e - MKS42c Motor Driver - NEMA17 Electric Step Motor Gripper 

<br>

<img src="https://github.com/cobang0111/centerpose_pick_and_place/assets/97373900/78fa09f0-9b75-409e-ae5f-51c75859d1cc" width="480">

<br>
<br>

[MKS42c Product Link](https://ko.aliexpress.com/item/1005003340856835.html?gatewayAdapt=glo2kor)

[Gripper Product Link](https://ko.aliexpress.com/item/1005002882854035.html?spm=a2g0o.detail.pcDetailTopMoreOtherSeller.11.61f8TtBeTtBebn&gps-id=pcDetailTopMoreOtherSeller&scm=1007.40000.327270.0&scm_id=1007.40000.327270.0&scm-url=1007.40000.327270.0&pvid=dbf953de-5016-4293-9428-c877adfc30d6&_t=gps-id:pcDetailTopMoreOtherSeller,scm-url:1007.40000.327270.0,pvid:dbf953de-5016-4293-9428-c877adfc30d6,tpp_buckets:668%232846%238113%231998&pdp_npi=4%40dis%21KRW%21250226%21237715%21%21%211305.80%211240.51%21%40214100e417063335498151432edfbc%2112000036801772264%21rec%21KR%21%21AB&utparam-url=scene%3ApcDetailTopMoreOtherSeller%7Cquery_from%3A)

<br>

## ✨Prerequisite
This repository assumes that you have already installed the docker container of isaac-ros and connected the Universal Robots (UR) with the Host PC and installed all the official ROS2 packages.
Also, MoveIt2 simple path planning of UR must be possible.
<br>
Please follow the step of below official isaac_ros_pose_estimation centerpose Github and QuickStart link.
<br>
[Github isaac_ros_pose_estimation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation)
<br>
[QuickStart isaac_ros_centerpose](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_centerpose/index.html#quickstart)

<br>

I have an issue when start the docker container.
If you so, Please start docker container using below command.
```bash
docker run --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it --privileged --net=host isaac_ros_dev-x86_64:latest
```

<br>

After you test the example of isaac_ros_pose_estimation centerpose, 
Install Intel RealSense SDK in docker container
```bash
xhost +local:root

docker start <container ID>

docker exec -it <container ID> bash

cd ~

git clone https://github.com/IntelRealSense/librealsense.git

sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

cd librealsense

sudo ./scripts/setup_udev_rules.sh

./scripts/patch-realsense-ubuntu-lts-hwe.sh

echo 'hid_sensor_custom' | sudo tee -a /etc/modules

mkdir build && cd build

cmake ../ -DBUILD_EXAMPLES=true

make -j$(nproc)

sudo make install

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc

source ~/.bashrc

realsense-viewer
```

<br>

Install ROS2 Intel Realsense under /workspace/isaac_ros-dev/src directory
<br>
https://github.com/IntelRealSense/realsense-ros
```bash
cd /workspace/isaac_ros-dev/src

git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

cd ..

colcon build
```

<br>

Install ROS2 UR Driver under /workspace/isaac_ros-dev/src directory
You must follow the step and install ROS2 humble version
<br>
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
```bash
sudo apt-get install ros-humble-ur

sudo apt install python3-colcon-common-extensions python3-vcstool

cd /workspace/isaac_ros-dev

git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver

vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos

rosdep update

rosdep install --ignore-src --from-paths src -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
<br>

Now, You need to modify urdf and mesh file of D435
Before start the below step, please check the PointCloud2 in rviz topic using below command in 2 terminal.

```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=848x480x30 depth_module.profile:=848x480x30 pointcloud.enable:=true
```
```bash
rviz2
```

If you have the problem that the camera direction and pointcloud direction are not matched,
rotate mesh file which locate /workspace/isaac_ros-dev/src/realsense-ros/realsense2_description/meshes/d435.dae using Blender

You can modify the  /workspace/isaac_ros-dev/src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro file like below
You need to change the detail size and the location of links
```xml
    <!-- RealSense Camera Link -->
    <link name="camera_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file:///workspace/isaac_ros-dev/src/realsense-ros/realsense2_description/meshes/d435_rotate.dae"/>
          <!--<box size="0.025 0.09 0.025"/>-->
        </geometry>
        <!--<material name="black"/>-->
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file:///workspace/isaac_ros-dev/src/realsense-ros/realsense2_description/meshes/d435_rotate.dae"/>
          <!--<box size="0.025 0.09 0.025"/>-->
        </geometry>
        <!--<material name="black"/>-->
      </collision>
    </link>

   <!-- Joint connecting wrist_3_link and camera_link  -->
   <joint name="camera_joint" type="fixed">
     <parent link="wrist_3_link"/>
     <child link="camera_link"/>
     <origin xyz="-0.0389 -0.0389 0.037" rpy="0 4.71 0.785"/>
   </joint>

   <!-- Gripper Link -->
   <link name="gripper_link">
     <visual>
       <origin xyz="0 0 0" rpy="0 0 0"/>
         <geometry>
           <box size="0.185 0.095 0.060"/>
         </geometry>
         <material name="black"/>
     </visual>
     <collision>
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <geometry>
         <box size="0.185 0.095 0.060"/>
       </geometry>
       <material name="black"/>
     </collision>
   </link>

   <!-- Joint connecting wrist_3_link and gripper_link  -->
   <joint name="gripper_joint" type="fixed">
     <parent link="wrist_3_link"/>
     <child link="gripper_link"/>
     <origin xyz="0 0 0.093" rpy="0 4.71 0.785"/>
   </joint>
```


<br>

## ✨Install
Please clone this repository under /workspace/isaac_ros-dev/src
```bash
cd /workspace/isaac_ros-dev/src

git clone https://github.com/cobang0111/centerpose_pick_and_place.git

cd ..
```

And then update bashrc file and rebuild the packages
```bash
nano ~/.bashrc

source /opt/ros/humble/setup.bash
source /workspace/isaac_ros-dev/install/setup.bash

colcon build

source ~/.bashrc
```
You may need to modify the code `src/centerpose_pick_and_place.cpp` appropriately to your case!!!


<br>

## ✨Execution
1st terminal (Activating x server + starting docker + Executing intel realsense node)



```
xhost +local:root

docker start <containter-id>

docker exec -it <containter-id> bash

cd /workspace/isaac_ros-dev

ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=848x480x30 depth_module.profile:=848x480x30 pointcloud.enable:=true rgb_camera.color_format:=bgr8
```


<br>

2nd terminal (Republishing intel realsense image topic)

```
docker exec -it <container-id> bash

cd /workspace/isaac_ros-dev

ros2 run topic_tools relay /camera/camera/color/image_raw /image
```

<br>

3rd terminal (Republishing intel realsense camera_info topic)

```
docker exec -it <container-id> bash

cd /workspace/isaac_ros-dev

ros2 run topic_tools relay /camera/camera/depth/camera_info /camera_info
```

<br>

4th terminal (Activating isaac-ros centerpose)

```
docker exec -it <container-id> bash

cd /workspace/isaac_ros-dev

ros2 launch isaac_ros_centerpose isaac_ros_centerpose_triton.launch.py model_name:=centerpose_shoe model_repository_paths:=['/tmp/models']
```

<br>


5th terminal (Activating Rviz)

You need to modify ip and ur_type appropriately

```
docker exec -it <container-id> bash

cd /workspace/isaac_ros-dev

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.101 launch_rviz:=false
```

<br>


6th terminal (Activating Moveit)

```
docker exec -it <container-id> bash

cd /workspace/isaac_ros-dev

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true robot_ip:=192.168.1.101 reverse_ip:=192.168.1.102
```

<br>

7th terminal (Activating Shoe Organization)

```
docker exec -it <container-id> bash

cd /workspace/isaac_ros-dev

ros2 launch centerpose_pick_and_place centerpose_pick_and_place_launch.py
```

<br>
<br>





