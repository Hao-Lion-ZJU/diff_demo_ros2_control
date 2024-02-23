# diff_demo_ros2_control

## 安装

```bash
sudo apt install ros-${ROS_DISTRO}-ros2-control*
```
安装需要使用的一些功能包
```
sudo apt-get install ros-humble-robot-state-publisher ros-humble-xacro
```
# 食用
下载编译
```
mkdir ~/diff_test_ws -p && cd ~/diff_test_ws
git clone git@github.com:Hao-Lion-ZJU/diff_demo_ros2_control.git
mv diff_demo_ros2_control/ src
colcon build
```
启动launch，并开启RVIZ可视化
```
ros2 launch diff_test_control diff_test.launch.py '<gui>:=<true>'
```
开启键盘控制并将话题重映射
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/test_controller/cmd_vel
```

差速小车效果图，笔者使用键盘控制小车画圆。

![image-20240217032932018](https://lion-1324338403.cos.ap-shanghai.myqcloud.com/PicGo/image-20240217032932018.png)

---

# 前言

`ros2-control`是ROS2提供的一套规范、实时的机器人控制框架，它具有灵活的拓展性，可以有效降低代码的耦合，它给开发者提供相对统一的API，并且有许多现成的功能包可供我们调用。

但是很遗憾，由于国内ROS2的资料相对较少，我并没有找到国内比较系统的介绍ros2-control的教程，为此只能参考官方的英文教程，自己对着GitHub上的代码，慢慢了解，慢慢消化。最后想着整理出博客，记录下自己的学习过程，这个系列我将带大家熟悉ros2-control框架怎么使用，手把手写一个简单的两轮差速小车的控制器，希望能够对感兴趣的同学有所启迪。ros2-control官方也有差速小车的例子，但是很多地方写的过于复杂不利于新手学习，而且有些代码逻辑有我感到解释不清的地方。

**限笔者水平有限，以上若有不正确的地方，欢迎大家批评指正。**

![image-20240212213322632](https://lion-1324338403.cos.ap-shanghai.myqcloud.com/PicGo/image-20240212213322632.png)

# 我们为什么需要用ros2-control

相信大家在使用ROS编写机器人代码，更多时候是把ROS作为一个通讯框架和一个工具箱，我们可以用话题传输数据，用各种现成的功能包。在进行运动学、动力学解算，使用各种控制学方法时候，通常都是把逻辑自行编写到一个进程里面，封装成一个节点。这样的好处是比较自由，代码量少，可以快速搭建起来。但这样，我们很多轮子就要自己去造，而且不同程序员编写代码习惯不同，别人在接手你的工作时，学习架构的时间成本也大，最大的问题是非常不利于移植。

我举个例子，假如公司有程序员A写了个六自由度机械臂的控制代码，程序员B写了个差速小车的控制代码，现在领导让你把他们两个整合整合，做一台移动小车上带有机械臂。那你还需要看懂他们两个的程序架构，然后自己再搭建一个架构，把他们的融合进来，时间人力成本都比较大。

![image-20240211183119629](https://lion-1324338403.cos.ap-shanghai.myqcloud.com/PicGo/image-20240211183119629.png)

那么如果我们都使用统一的`ros2-control`框架，借助其高解耦特性，各类控制器或者硬件驱动都被抽象为了一个个插件，我们都可以轻松拿来复用，选择性的增加或者减少各模块，假如我们的电机更换了也只需复写少量代码即可。

别不信，当你跟我做完了整个项目再回头看时，你会惊讶于`ros2-control`框架漂亮的解耦设计，我们可以在线更换控制逻辑，这在我们调试时候极为方便。我们想完成上述例子的修改，甚至只需要修改几处urdf文件即可。比如我们更换个电机，也不需要把前人的整个代码架构搞清楚，只需要重新写一个硬件组件等等。

以上只是我个人的理解，大家也可以参考官网教程获取介绍:[Welcome to the ros2_control documentation!](https://control.ros.org/master/index.html)

还有官方的GitHub仓库:[ros-controls (github.com)](https://github.com/ros-controls)

这里我也分享一个油管上面讲的比较好的博主，英文好而且感兴趣的同学可以自行观看：[here](https://youtu.be/4QKsDf1c4hc?si=ElQ71NsBjqBAqKP2)

# ros2-control基础概念

首先，熟悉ros2-control框架必须知道4个概念，分别是**控制器管理器(Controller Manager)、硬件资源管理器(Resource Manager)、控制器(Controllers)和硬件组件(Hardware Component)**

**控制器管理器**：是一个类，也可以说是官方写好的一个功能包，可以实现对各类控制器的管理。控制器管理器会通过launch和yaml文件加载控制器，如果是实时内核还会自动帮我们提升控制器运行的优先级，还能够与终端命令行进行交互；

**硬件资源管理器**：管理各类硬件组件，根据URDF的描述文件决定加载哪些硬件组件

**控制器**：执行控制算法的具体逻辑，以插件的形式可以被控制器管理器动态加载

**硬件组件**：用于直接与硬件打交道的代码，也是以插件形式被资源管理器调用。有系统(system)、执行器(actuator)、传感器(sensor)三类,他们的行为被统一抽象为**read、write**

下面给出一张比较直观的理解图（图片从油管博主上搬运）：

![image-20240212232310897](https://lion-1324338403.cos.ap-shanghai.myqcloud.com/PicGo/image-20240212232310897.png)

以及官方的整个框架图，这张图在后续我们概念理解上面尤为重要，值得反复去看

!["UML Class Diagram"](https://control.ros.org/master/_images/uml_class_diagram.png)



为了更加深入的理解，建议大家可以在学习ros2-control框架之前，首先学习ROS2的基本概念和一些常用的工具，例如RVIZ、URDF等等

## 作者
- [机器小狗史努比](https://github.com/Hao-Lion-ZJU)-Hao Lion

如果对其中有问题不清楚，欢迎大家给我私信留言或者加入我创建的一个小的交流群一起学习交流**QQ-536788289**

创作不易，如果对您有帮助，麻烦点个star★★★加个关注吧！我会继续努力分享更多ROS2小知识