# ROS-Video-Recorder
A ROS based systems used to capture images sent over ROS topics and compiles them into the specified video format. 
Tested with specially made baumer cameras. Intergrates with any robotic sensor system using ROS to operate. 

**How to Use:**
  * Clone Repo: (using HTTPS) git clone https://github.com/ijeriomit/ROS-Video-Recorder.git (using SSH) git clone git@github.com:ijeriomit/ROS-Video-Recorder.git
  * Build the repo: catkin install ROS-Video-Recorder using catkin (on Linux) or catkin_make for Windows Users 
  * Running the Mock Camera: To launch a mock version of a compatible camera use ```rosrun robot_video_recorder mock_camera```
  * Running the Recorder: To launch the recording system run ```rosrun robot_video_recorder video_recorder``` pass arguments here for: image_height, image_width, folder_path, etc. 
  ex. ```rosrun robot_video_recorder video_recorder image_height:= 1920 image_width:= 1080```
