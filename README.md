# Minimal Grasp Environment (最小化抓取环境)

本项目旨在提供一个尽可能简化的 Franka Panda 抓取环境，方便快速复现 **ROS + Gazebo + MoveIt** 下的抓取实验。
## 开发者宣言
<div style="text-align: center;"> 
我去除了大部分bug的腥味<br>
但我保留了一部分<br>
就是为了让初学者意识到你面对的是ROS<br>
而不是你上幼儿园时算的加法题
</div>



---

## 构建步骤

### 1. 编译工程
建议在ubuntu18.04版本编译运行，在~文件夹（家目录）运行git命令下载此项目
```bash
git clone https://github.com/zlwq/minimal_grasp.git 
```
进入工作空间，然后编译：
```bash
cd minimal_grasp
catkin_make
```
在编译过程中，可能会因为缺少依赖而报错。
此时可以根据报错信息，利用 AI 或搜索引擎 来提示需要安装的依赖，例如：
```bash
sudo apt install ros-melodic-<package-name>
```
逐步补齐所有依赖后即可完成构建。

### 2. 配置环境变量
在 ~/.bashrc 中添加以下行，使 Gazebo 能找到自定义模型：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zlwq/minimal_grasp/src/panda_demo/urdf
```
保存后刷新：

```bash
source ~/.bashrc
```
### 3. 加载工作空间环境
构建完成后，在 ~/.bashrc 或手动执行：

```bash
source ~/minimal_grasp/devel/setup.bash
```
### 4、打开launch文件和对应的banana_grasp
完成以上步骤后，即可在 Gazebo 中启动最小化 Panda 抓取环境，并基于此进行抓取复现。
```bash
roslaunch panda_moveit_config demo_gazebo.launch  
```
等待的时间需要长一点，直到rviz界面的机械臂加载完毕后，运行以下命令开始抓取喵
```bash
rosrun banana_grasp_pkg banana_grasp
```
### 注意事项
本配置为最小化抓取环境，因此许多非必要功能和插件已被精简，建议使用ubuntu18.04版本进行构建复现。遇到任何问题可在issue板块发布。

另外，franka_ros的franka_gazebo的.h头文件已经删去，因为ubuntu18.04可以下载到ros内置的franka插件，apt-install后系统自动会将对应的头文件安装在opt/ros/melodic/include里面。

如需额外功能，请在编译过程中根据报错提示自行安装缺失的 ROS 依赖包。

### 致谢
本环境基于 franka_ros和官方教程moviet_tutorial 改造而来，感谢原作者的开源贡献。
