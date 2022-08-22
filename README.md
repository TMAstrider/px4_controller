# Project_August
This is the source code that was created for autonomous delivery and cv detection.


# Step 001: Make sure that you have the PX4, GAZEBO, ROS, MAVROS simulation environment set up!
If not, please check the site: [XTDrone Manual](https://www.yuque.com/xtdrone/manual_cn). You can check XTDrone Environment Set Up for More infomation. XD

# Step 002: Make sure that you have ROS basics 
Here's tutorial:[ROS Tutotial EN](http://wiki.ros.org/ROS/Tutorials) and [ROS Tutorial CN](http://wiki.ros.org/cn/ROS/Tutorials). 




# Step 003: Implement that offboard example on your own:
Please check the site: [PX4 offboard example](https://docs.px4.io/main/en/ros/mavros_offboard_python.html). 
Prerequisites are MAVROS ROS and Python basics. 
The topics can be found in [MAVROS Topics](http://wiki.ros.org/mavros). 
GAAS tutorial E01, which is an optional is also a good choice for implementing offboard example. 
GAAS E01 ->[GAAS E01](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/wu-ren-ji-zi-dong-jia-shi-xi-lie-offboard-kong-zhi-yi-ji-gazebo-fang-zhen)


# Step 004: Write your own ROS Node to control your UAV however you want
Supposing that GAAS had written an API for controlling PX4, I guess you must make sure that you totally understand how subscriber, publisher, and topics work and make some changes to python file `px4_mavros_run.py` and `commander.py`.


# Step 005 : Write your own ROS Node to implement the UAV tracking Node which contains a bunch of subscribers publishers, and an important callback Function.(Both Python and C++ are allowed ) (This part is a simulation)
You can simply check the c++ file above for your reference. You cna refer to this [blog](https://blog.liujiawei.xyz/2021/10/23/PX4%E6%97%A0%E4%BA%BA%E6%9C%BA-Gazebo%E4%BB%BF%E7%9C%9F%E5%AE%9E%E7%8E%B0%E7%A7%BB%E5%8A%A8%E7%89%A9%E4%BD%93%E7%9A%84%E8%B7%9F%E8%B8%AA/#%E6%8E%A7%E5%88%B6%E6%97%A0%E4%BA%BA%E6%9C%BA%E8%B7%9F%E8%B8%AA%E8%BF%90%E5%8A%A8%E5%B0%8F%E8%BD%A6)

# Step 006: Make sure that you have Yolov5 Environment Set Up!
If not, please check: [YOLOv5](https://github.com/ultralytics/yolov5). 
if you have detect.py run, then you can go to Step 007.

# Step 007: Integrate ROS with YOLOv5
please make sure that you can handle the things above and continue this part.
You will have your UAC camera, which is the frontend of you computer, running, and check [this repository](https://github.com/qianmin/yolov5_ROS). GOOD LUCK!


# Step 008: Write your own Callback Function which sends the diffs 
