# Intelligent-Robotics-Project-1
CS 4023 Intro to Intelligent Robotics Project 1 Repository

## Steps to Run The Project 

## #1 Clone the Github repository in a suitable location with
`git clone https://github.com/Kichlids/Intelligent-Robotics-Project-1.git`
Or unpack the zipped project submitted into a suitable location

## #2 CD into the project workspace
`cd /<Path to project>/Intelligent-Robotics-Project-1/robot-ws`

## #3 Execute Catkin make
`catkin_make`

## #4 Source the setup.bash file
`source devel/setup.bash`

## #5 Execute the main launch file
`roslaunch robot_control robot_control.launch`   
Use the WASD keys to control the robot in the same terminal that you just ran the previous command.

## #6 Execute these commands to view the robot mapping the room
In a separate terminal launch rviz   
`roslaunch turtlebot_rviz_launchers view_navigation.launch`   
After mapping you can save the map   
`rosrun map_server map_saver -f <your map name>`   
Execute this command to run autonomous pathing with your map (Don’t close the other terminals or programs)   
`roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<full path to your map YAML file>`
