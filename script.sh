# Bringup
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# script.sh
## Straight line
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.2, y: 0.0, z: 0.0}}" & 
sleep 15 
rostopic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.2, y: 0.0, z: 0.0}}" & # Stop

## Circular Motion
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.24, y: 0.0, z: 0.0}, angular: {x: 0.2, y: 0.0, z: -0.4}}" &
sleep 20
rostopic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.2, y: 0.0, z: 0.0}}" & # Stop