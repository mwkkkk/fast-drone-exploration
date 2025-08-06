This document is an extension of [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL), focusing on real-world flight.

# My Work：
1.Used [Px4Ctrl](about:blank) as the controller to track the trajectories published by FUEL in real-world flight experiments.

2.[FUEL](about:blank) requires a `sensor_pose_topic` as input. I converted [VINS](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) `/vins_fusion/imu_propagate` topic to match the required format. If using `/vins_fusion/camera_pose`, an additional coordinate frame transformation would be necessary.



# Result：
The video results of the real-world experiments can be viewed [here](https://www.bilibili.com/video/BV11ktnzpEt6?spm_id_from=333.788.recommend_more_video.0&vd_source=18e58623b724ce37f29dfe4496de77e9)(Bilibili).



# Preparation:
Follow this [link](https://github.com/ZJU-FAST-Lab/Fast-Drone-250) to finish.

1.Set up the environment on the onboard computer, including RealSense and MAVROS.

2.VINS Parameter Configuration



# How to use：
1.Start mavros、camera and vins.

`sh shfiles/rspx4.sh`



2.Check the position data from `/vins_fusion/imu_propagate` to verify if there are any issues.

`rostopic echo /vins_fusion/imu_propagate`



3.If there is no problem, start Px4Ctrl

`roslaunch px4ctrl run_ctrl.launch`



4.Convert the message type of `/vins_fusion/imu_propagate` and republish it as a new topic.

`rosrun exploration_manager odom_to_pose`



5.Start FUEL

`roslaunch exploration_manager exploration_in_exp.launch`



6.Start takeoff

`sh shfiles/takeoff.sh`



Do not run RViz on the onboard computer. If visualization is needed, use ROS multi-machine communication to view it from another computer.

rviz：`roslaunch exploration_manager rviz.launch`



# Limitation：
There is an issue with trajectory visualization: as observed in the video, the trajectory keeps flickering and eventually disappears.(To Solve)



# <font style="color:rgb(31, 35, 40);">Reference Projects</font>
1.[FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)

The planning module of the drone is from this project.

2.[<font style="color:rgb(31, 35, 40) !important;">Fast-Drone-250</font>](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)

<font style="color:rgb(31, 35, 40) !important;">The drone platform was constructed using this project, including the controller, which is also derived from it.</font>

<font style="color:rgb(31, 35, 40) !important;">3.</font>[VINS](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

Location from this project.

4.[<font style="color:rgb(31, 35, 40) !important;">Fast-Exploration</font>](https://github.com/XXLiu-HNU/Fast-Exploration)

FUEL Simulation Environment<font style="color:rgb(31, 35, 40) !important;">. It is important to successfully run the simulation environment before attempting real-world flight experiments.</font>





