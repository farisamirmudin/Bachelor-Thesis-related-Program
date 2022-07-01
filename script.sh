# Bringup
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# script.sh
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.2, y: 0.0, z: 0.0}}" & # linear x: 0.24, angular z: -0.4 if circular motion
sleep 15 # 20 if ciruclar motion
rostopic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.2, y: 0.0, z: 0.0}}" & # Stop