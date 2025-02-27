## 重要提示:
&ensp;&ensp;由于机械臂通信格式修改, 建议在***2019年6月前发货***的xArm 早期用户尽早 ***升级*** 控制器固件程序，这样才能在以后的更新中正常驱动机械臂运动以及使用最新开发的各种功能。请联系我们获得升级的详细指示。 当前ROS库主要的分支已不支持旧版本，先前版本的ROS驱动包还保留在 ***'legacy'*** 分支中, 但不会再有更新。    

# 目录:  
* [1. 简介](#1-简介)
* [2. 更新记录](#2-更新记录)
* [3. 准备工作](#3-准备工作)
* [4. 开始使用'xarm_ros'](#4-开始使用xarm_ros)
* [5. 代码库介绍及使用说明](#5-代码库介绍及使用说明)
    * [5.1 xarm_description](#51-xarm_description)  
    * [5.2 xarm_gazebo](#52-xarm_gazebo)  
    * [5.3 xarm_controller](#53-xarm_controller)  
    * [5.4 xarm_bringup](#54-xarm_bringup)  
    * [5.5 ***xarm7_moveit_config (Updated)***](#55-xarm7_moveit_config)  
    * [5.6 xarm_planner](#56-xarm_planner)  
    * [5.7 ***xarm_api/xarm_msgs***](#57-xarm_apixarm_msgs)  
        * [5.7.1 使用ROS Service启动 xArm (***后续指令执行的前提***)](#使用ros-service启动-xarm)  
        * [5.7.2 关节空间和笛卡尔空间运动指令的示例](#关节空间和笛卡尔空间运动指令的示例)
        * [5.7.3 I/O 操作](##io-操作)  
        * [5.7.4 获得反馈状态信息](#获得反馈状态信息)  
        * [5.7.5 关于设定末端工具偏移量](#关于设定末端工具偏移量)  
        * [5.7.6 清除错误](#清除错误)  
		* [5.7.7 机械爪控制(***new***)](#机械爪控制)  
* [6. 模式切换(***new***)](#6-模式切换)
    * [6.1 模式介绍](#61-模式介绍)
    * [6.2 切换模式的正确方法](#62-切换模式的正确方法)


# 1. 简介：
   &ensp;&ensp;此代码库包含xArm模型文件以及相关的控制、规划等示例开发包。开发及测试使用的环境为 Ubuntu 16.04 + ROS Kinetic Kame。
   ***以下的指令说明是基于xArm7, 其他型号用户可以在对应位置将'xarm7'替换成'xarm6'或'xarm5'***

# 2. 更新记录：
   此代码库仍然处在早期开发阶段，新的功能支持、示例代码，bug修复等等会保持更新。  
   * 添加xArm 7(旧版)描述文档，3D图形文件以及controller示例，用于进行ROS可视化仿真模拟。
   * 添加MoveIt!规划器支持，用于控制Gazebo/RViz模型或者xArm真机，但二者不可同时启动。
   * 由ROS直接控制xArm真机的相关支持目前还是Beta版本，用户使用时应尽量小心，我们会尽快完善。
   * 添加 xArm hardware interface 并在驱动真实机械臂时使用 ROS position_controllers/JointTrajectoryController。
   * 添加 xArm 6 仿真和真机控制支持。
   * 添加 xArm 机械爪仿真模型。

# 3. 准备工作

## 3.1 安装依赖的模块
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> （如果使用Gazebo模拟器）  
   ros_control: <http://wiki.ros.org/ros_control> (记得选择您使用的 ROS 版本)  
   moveit_core: <https://moveit.ros.org/install/>  
   
## 3.2 完整学习相关的官方教程
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

## 3.3 如果使用Gazebo: 请提前下载好 'table' 3D 模型
&ensp;&ensp;这个模型在Gazebo demo中会用到。在Gazebo仿真环境中, 在model database列表里寻找 'table', 并将此模型拖入旁边的3D环境中. 通过这个操作，桌子的模型就会自动下载到本地。

# 4. 开始使用'xarm_ros'
   
## 4.1 生成catkin workspace. 
   &ensp;&ensp;如果您已经有了自己的catkin工作区，请跳过此步往下进行。
   按照[这里](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)的教程生成catkin_ws。 
   请留意本文档已假设用户继续沿用 '~/catkin_ws' 作为默认的catkin工作区地址。

## 4.2 获取代码包
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git
   ```

## 4.3 安装其他依赖包:
   ```bash
   $ rosdep update
   $ rosdep check --from-paths . --ignore-src --rosdistro kinetic
   ```
   请将 'kinetic' 修改为您在使用的ROS版本。如有任何未安装的依赖包列出，请执行以下命令自动安装:  
   ```bash
   $ rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
   ```
   同样的，请将 'kinetic' 修改为您在使用的ROS版本。  

## 4.4 编译代码
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.5 执行配置脚本
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
如果在您的 ~/.bashrc中已经有了以上语句，直接运行:
```bash
$ source ~/.bashrc
```
## 4.6 在RViz环境试用:
```bash
$ roslaunch xarm_description xarm7_rviz_display.launch
```
## 4.7 如果已安装Gazebo,可以执行demo查看效果
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true]
   ```
&ensp;&ensp;指定'run_demo'为true时Gazebo环境启动后机械臂会自动执行一套编好的循环动作。 这套简单的command trajectory写在xarm_controller\src\sample_motion.cpp. 这个demo加载的控制器使用position interface（纯位置控制）。

# 5. 代码库介绍及使用说明
   
## 5.1 xarm_description
   &ensp;&ensp;包含xArm描述文档, mesh文件和gazebo plugin配置等等。 不推荐用户去修改urdf描述因为其他ros package对其都有依赖。

## 5.2 xarm_gazebo
   &ensp;&ensp;Gazebo world 描述文档以及仿真launch启动文档。用户可以在world中修改添加自己需要的模型与环境。

## 5.3 xarm_controller
   &ensp;&ensp;xarm使用的Controller配置, 硬件接口，轨迹指令源文件, 脚本以及launch文件。 用户可以基于这个包开发或者使用自己的package。***注意*** xarm_controller/config里面定义好的position/effort控制器仅用作仿真的示例, 当控制真实机械臂时只提供position_controllers/JointTrajectoryController接口。用户可以根据需要添加自己的controller, 参考: http://wiki.ros.org/ros_control (controllers)

## 5.4 xarm_bringup  
&ensp;&ensp;内含加载xarm driver的启动文件，用来控制真实机械臂。  

## 5.5 xarm7_moveit_config
&ensp;&ensp;
   部分文档由moveit_setup_assistant自动生成, 用于Moveit Planner和Rviz可视化仿真。如果已安装MoveIt!,可以尝试跑demo： 
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```

#### Moveit!图形控制界面 + Gazebo 仿真环境:  
   首先执行:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   然后在另一个终端运行:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   如果您在Moveit界面中规划了一条满意的轨迹, 点按"Execute"会使Gazebo中的虚拟机械臂同步执行此轨迹。

#### Moveit!图形控制界面 + xArm 真实机械臂:
   首先, 检查并确认xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<控制盒的局域网IP地址>
   ```
   检查terminal中的输出看看有无错误信息。如果启动无误，您可以将RViz中通过Moveit规划好的轨迹通过'Execute'按钮下发给机械臂执行。***但一定确保它不会与周围环境发生碰撞！***  

#### Moveit!图形控制界面 + 安装了UFACTORY机械爪的xArm6真实机械臂:  
   首先, 检查并确认xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm6_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   如果使用了我们配套的机械爪(xArm gripper), 最好可以使用这个package，因为其中的配置会让Moveit在规划无碰撞轨迹时将机械爪考虑在内。 
  

## 5.6 xarm_planner:
这个简单包装实现的规划器接口是基于 Moveit!中的 move_group interface, 可以使用户通过service指定目标位置进行规划和执行。 这部分的详细使用方法请阅读[xarm_planner包](./xarm_planner)的文档。  
#### 启动 xarm simple motion planner 控制 xArm 真实机械臂:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<控制盒的局域网IP地址> robot_dof:=<7|6|5>
```
'robot_dof'参数指的是xArm的关节数目 (默认值为7)。  

## 5.7 xarm_api/xarm_msgs:
&ensp;&ensp;这两个package提供给用户不需要自己进行轨迹规划(通过Moveit!或xarm_planner)就可以控制真实xArm机械臂的ros服务, xarm自带的控制盒会进行轨迹规划。 请 ***注意*** 这些service的执行并不通过面向'JointTrajectoryController'的hardware interface。当前支持三种运动命令（ros service同名）:  
* move_joint: 关节空间的点到点运动, 用户仅需要给定目标关节位置，运动过程最大关节速度/加速度即可。 
* move_line: 笛卡尔空间的直线轨迹运动，用户需要给定工具中心点（TCP）目标位置以及笛卡尔速度、加速度。  
* move_lineb: 圆弧交融的直线运动，给定一系列中间点以及目标位置。 每两个中间点间为直线轨迹，但在中间点处做一个圆弧过渡（需给定半径）来保证速度连续。
另外需要 ***注意*** 的是，使用以上三种service之前，需要通过service依次将机械臂模式(mode)设置为0，然后状态(state)设置为0。这些运动指令的意义和详情可以参考产品使用指南。除此之外还提供了其他xarm编程API支持的service调用, 对于相关ros service的定义在 [xarm_msgs目录](./xarm_msgs/)中。 

#### 使用ROS Service启动 xArm:

&ensp;&ensp;首先启动xarm7 service server, 以下ip地址只是举例:  
```bash
$ roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.128
```
&ensp;&ensp;然后确保每个关节的控制已经使能, 参考[SetAxis.srv](/xarm_msgs/srv/SetAxis.srv):
```bash
$ rosservice call /xarm/motion_ctrl 8 1
```
&ensp;&ensp;在传递任何运动指令前，先***依次***设置正确的机械臂模式(0: POSE)和状态(0: READY), 参考[SetInt16.srv](/xarm_msgs/srv/SetInt16.srv):    
```bash
$ rosservice call /xarm/set_mode 0

$ rosservice call /xarm/set_state 0
```

#### 关节空间和笛卡尔空间运动指令的示例:
&ensp;&ensp;以下三个运动命令使用同类型的srv request: [Move.srv](./xarm_msgs/srv/Move.srv)。 比如，调用关节运动命令，最大速度 0.35 rad/s，加速度 7 rad/s^2:  
```bash
$ rosservice call /xarm/move_joint [0,0,0,0,0,0,0] 0.35 7 0 0
```
&ensp;&ensp;调用笛卡尔空间指令，最大线速度 200 mm/s，加速度为 2000 mm/s^2:
```bash
$ rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
```
&ensp;&ensp;调用回原点服务 (各关节回到0角度)，最大角速度 0.35 rad/s，角加速度 7 rad/s^2:  
```bash
$ rosservice call /xarm/go_home [] 0.35 7 0 0
```

#### 工具 I/O 操作:
&ensp;&ensp;我们在机械臂末端提供了两路数字、两路模拟输入信号接口，以及两路数字输出信号接口。  
##### 1. 同时获得2个数字输入信号状态的方法:  
```bash
$ rosservice call /xarm/get_digital_in
```
##### 2. 获得某一模拟输入信号状态的方法: 
```bash
$ rosservice call /xarm/get_analog_in 1  (最后一个参数：端口号，只能是1或者2)
```
##### 3. 设定某一个输出端口电平的方法:
```bash
$ rosservice call /xarm/set_digital_out 2 1  (设定输出端口2的逻辑为1)
```
&ensp;&ensp;注意检查这些service返回的"ret"值为0，来确保操作成功。

#### 获得反馈状态信息:
&ensp;&ensp;如果通过运行'xarm7_server.launch'连接了一台xArm机械臂，用户可以通过订阅 ***"/xarm_states"*** topic 获得机械臂当前的各种状态信息， 包括关节角度、工具坐标点的位置、错误、警告信息等等。具体的信息列表请参考[RobotMsg.msg](./xarm_msgs/msg/RobotMsg.msg).  
&ensp;&ensp;另一种选择是订阅 ***"/joint_states"*** topic, 它是以[JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html)格式发布数据的, 但是当前 ***只有 "position" 是有效数据***; "velocity" 是没有经过任何滤波的基于相邻两组位置数据进行的数值微分, 因而只能作为参考，我们目前还不提供 "effort" 的反馈数据.
&ensp;&ensp;基于运行时性能考虑，目前以上两个topic的数据更新率固定为 ***10Hz***.  

#### 关于设定末端工具偏移量:  
&ensp;&ensp;末端工具的偏移量可以也通过'/xarm/set_tcp_offset'服务来设定,参考下图，请注意这一坐标偏移量是基于 ***原始工具坐标系*** (坐标系B)描述的，它位于末端法兰中心，并且相对基坐标系(坐标系A)有（PI, 0, 0)的RPY旋转偏移。
![xArmFrames](./doc/xArmFrames.png)  
&ensp;&ensp;例如：
```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp;这条命令设置了基于原始工具坐标系(x = 0 mm, y = 0 mm, z = 20 mm)的位置偏移量，还有（0 rad, 0 rad, 0 rad)的RPY姿态偏移量。***如果需要请在每次重新启动/上电控制盒时设定一次正确的偏移量，因为此设定会掉电丢失。***  

#### 清除错误:
&ensp;&ensp;有时控制器会因为掉电、位置或速度超限、规划失败等原因汇报错误，遇到这一状态需要手动解除。具体的错误代码可以在topic ***"/xarm_states"*** 的信息中找到。 
```bash
$ rostopic echo /xarm_states
```
&ensp;&ensp;如果'err'字段数据为非零，需要对照用户手册找到原因并设法解决问题。之后，这一错误状态可以通过调用服务 ***"/xarm/clear_err"*** 清除：  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;调用此服务之后 ***务必再次确认err状态信息*** , 如果它变成了0, 说明问题清除成功，否则请再次确认问题是否成功解决。清除成功之后， 记得 ***将robot state设置为0*** 以便使机械臂可以执行后续指令。  

#### 机械爪控制:
&ensp;&ensp;如果已将xArm Gripper (UFACTORY出品) 安装至机械臂末端，则可以使用如下的service来操作和检视机械爪:  
1. 首先使能xArm机械爪并设置抓取速度, 如果成功，'ret'返回值为0。正确的速度范围是在***1到5000之间***，以1500为例:  
```bash
$ rosservice call /xarm/gripper_config 1500
```
2. 给定xArm机械爪位置（打开幅度）指令执行，如果成功，'ret'返回值为0。正确的位置范围是***0到850之间***, 0为关闭，850为完全打开，以500为例:  
```bash
$ rosservice call /xarm/gripper_move 500
```
3. 获取当前xArm机械爪的状态（位置和错误码）:
```bash
$ rosservice call /xarm/gripper_status
```
&ensp;&ensp;如果错误码不为0，请参考使用说明书查询错误原因，清除错误同样可使用上一节的clear_err service。

# 6. 模式切换
&ensp;&ensp;xArm 在不同的控制方式下可能会工作在不同的模式中，当前的模式可以通过topic "/xarm_states" 的内容查看。在某些情况下，需要用户主动切换模式以达到继续正常工作的目的。

### 6.1 模式介绍

&ensp;&ensp; ***Mode 0*** : 基于xArm controller规划的位置模式；   
&ensp;&ensp; ***Mode 1*** : 基于外部轨迹规划器的位置模式；  
&ensp;&ensp; ***Mode 2*** : 自由拖动(零重力)模式。  

&ensp;&ensp;***Mode 0*** 是系统初始化的默认模式，当机械臂发生错误(碰撞、过载、超速等等),系统也会自动切换到模式0。并且对于[xarm_api](./xarm_api/)包和[SDK](https://github.com/xArm-Developer/xArm-Python-SDK)中提供的运动指令都要求xArm工作在模式0来执行。***Mode 1*** 是为了方便像 Moveit! 一样的第三方规划器绕过xArm控制器的规划去执行轨迹。 ***Mode 2*** 可以打开自由拖动模式, 机械臂会进入零重力状态方便拖动示教, 但需注意在进入模式2之前确保机械臂安装方式和负载均已正确设置。

### 6.2 切换模式的正确方法:  
&ensp;&ensp;如果在执行Moveit!规划的轨迹期间发生碰撞等错误, 为了安全考虑，当前模式将被自动从1切换到0, 同时robot state将变为 4 (错误状态)。这时即使碰撞已经解除，机械臂在重新回到模式1之前也无法执行任何Moveit!或者servoj指令。请依次按照下列指示切换模式:  

&ensp;&ensp;(1) 确认引发碰撞的物体已经被移除；  
&ensp;&ensp;(2) 清除错误:  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;(3) 切换回想要的模式 (以Mode 2为例), 之后 set state 为0（Ready状态）:
```bash
$ rosservice call /xarm/set_mode 2

$ rosservice call /xarm/set_state 0
```
&ensp;&ensp;以上操作同样可用相关的xArm SDK函数实现.
