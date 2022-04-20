# lane following
This is the lab resources for SUSTech EE346.

# Usage

## 1. Clone the source code
  cd ~/catkin_ws/src
  
  git clone https://github.com/tianyanyushui/lane_following
  
## 2. Catkin make the lane following package
  cd ..
  
  catkin_make

## 3. Add course models
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models
   
## 4. Launch the gazebo map
   source ~/catkin_ws/devel/setup.bash
   
   roslaunch lane_following race_track.launch 

## 5. Run lane following python node
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following.py
   
![image](https://github.com/zhaojieting/linefollowing/blob/main/data/demo.gif)

## 6. Get bird’s-eye-view
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following_part2.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following_part2.py
   
## 7. Stop Sign with Aruco Marker
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following_part3.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash 
  
   rosrun lane_following lane_following_part3.py

