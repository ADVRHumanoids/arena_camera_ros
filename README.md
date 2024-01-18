
# Useful links

https://support.thinklucid.com/helios2plus-htp003s/
https://support.thinklucid.com/kb/camera-features/
https://support.thinklucid.com/helios2-hlt003s/#7050
https://support.thinklucid.com/knowledgebase/pixel-formats/#coord3D_abcy16

Forked from: https://github.com/lucidvisionlabs/arena_camera_ros

# Dependencies
- ArenaSDK

- Copy "camera.yaml" in ~/.ros/arena_camera /config/

# How to run
- roscore
- rviz -d ~/helios_ws/arena_camera_ros/src/arena_cloud/config/rviz.rviz
- [Launch Only Camera --> Images] roslaunch arena_camera arena_camera_node.launch
- [Launch camera and cloud] roslaunch arena_cloud arena_cloud.launch

# In case of linking overwrite for Gazebo (remove ArenaSDK linking)
- https://stackoverflow.com/questions/73051114/gazebo11-symbol-lookup-error-libavfilter-so-7d

- sudo rm -rf /etc/ld.so.conf.d/Arena_SDK.conf
- sudo apt-get install --reinstall libavfilter7
