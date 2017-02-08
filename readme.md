# Traffic Cone Finder code for RoboMagellan

## Jupyter Notebooks
- Detect cones in a video feed
- Publish heading for the cones

## ROS
Build the ROS Code:
* `cd ~/catkin_ws`, assuming the typical ROS install.
* `catkin_make`


To run ROS node
* `rosrun cone_finder detect_cones.py [-d] [r]`
* # -d/--debug is optional and will show real time video with cones marked
* # -r/--use_ros_topic to use published topics from realsense_camera node
* Topic published:
    * /cone_finder/locations
    * /cone_finder/rgbImage
    * /cone_finder/depthImage
* See cone_finder/msg/locations_msg.msg for details

- [ROS node which detect cones using ROS realsense_camera node or camera video](ROS/readme.md)

## License
- [Repository License](License.md)

