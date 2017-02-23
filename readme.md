# Traffic Cone Finder and Cone Seeker code for RoboMagellan

## ROS
Build the ROS Code:
* `cd ~/catkin_ws`, assuming the typical ROS install.
* `catkin_make`

## Cone Seeker
This ros node will use the messages published by cone_finder node and publish 
mavros/rc/override messages for throttle and steering. To use this node:
* `roslaunch cone_finder cone_seeker.launch|cone_seeker_video.launch`
* # Use cone_seeker.launch to run this node without video
* # Use cone_seeker_video.launch to run this node with video
Both launch files will invoke cone_finder node.

## Cone Finder
This ros node will detect cones in images published by realsense_camera node or
a video feed or images in a directory. To run as ROS node, use:
* `roslaunch cone_finder cone_finder.launch|cone_finder_video.launch`
* # Use cone_finder.launch to run this node without publishing image topics
* # Use cone_finder_video.launch to run this node and publish image topics
The launch files will start the realsense_camera node automatically.

Alternatively, use:
* `rosrun cone_finder detect_cones.py [-d] [r] [-p]`
* # -d/--debug is optional and will show real time video with cones marked
* # -r/--use_ros_topic to use published topics from realsense_camera node
* # -p/--publish_images to control published rgbImage and depthImage topics

Topic published:
    * /cone_finder/locations
    * /cone_finder/rgbImage
    * /cone_finder/depthImage
* See cone_finder/msg/locations_msg.msg for details

## License
- [Repository License](License.md)

