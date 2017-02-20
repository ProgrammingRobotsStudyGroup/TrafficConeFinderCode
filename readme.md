# Traffic Cone Finder code for RoboMagellan

## ROS
Build the ROS Code:
* `cd ~/catkin_ws`, assuming the typical ROS install.
* `catkin_make`

To run ROS node
* `roslaunch cone_finder cone_finder.launch|cone_finder_video.launch`
* # Use cone_finder.launch to run this node without publishing image topics
* # Use cone_finder_video.launch to run this node and publish image topics
* Or `rosrun cone_finder detect_cones.py [-d] [r] [-p]`
* # -d/--debug is optional and will show real time video with cones marked
* # -r/--use_ros_topic to use published topics from realsense_camera node
* # -p/--publish_images to control published rgbImage and depthImage topics
* Topic published:
    * /cone_finder/locations
    * /cone_finder/rgbImage
    * /cone_finder/depthImage
* See cone_finder/msg/locations_msg.msg for details

## License
- [Repository License](License.md)

