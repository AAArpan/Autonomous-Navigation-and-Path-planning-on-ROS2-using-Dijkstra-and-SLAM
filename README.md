# Autonomous-Navigation-and-Path-planning-on-ROS2-using-Dijkstra-and-SLAM
This is the implementation of an autonomous navigation project using TurtleBot3 in a custom Gazebo world. The project integrates ROS 2 Humble, SLAM Toolbox, and a custom Dijkstra algorithm for path planning. The goal is to simulate a robot navigating through a maze-like environment, avoiding obstacles, and reaching its destination efficiently.

# FEATURES
1. SLAM (Simultaneous Localization and Mapping): Generate maps in real-time using the SLAM Toolbox.
2. Custom Gazebo World: A maze world built in Gazebo for simulation.
3. Custom Path Planning: Implemented Dijkstra’s algorithm to find the shortest path in the mapped environment.
4. ROS 2 Navigation Stack: Used for obstacle avoidance and path following.
5. Real-time Visualization: RViz is used to visualize the robot, map, and path.

# SETUP
```bash
git clone https://github.com/your-username/your-repo.git
cd your-repo
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-slam-toolbox
export TUTRTLEBOT3_MODEL=burger
```
Create a world on Gazebo and save it. Copy and paste the path in turtlebot3_world.launch.py

# RUN THE PROJECT
1. Launch the custom world map on Gazebo
```bash
ros2 launch turtlbot3_custom turtlebot3_world.launch.py use_sim_time:=true
```
2. RUN SLAM to create a 2-D MAP
We have to move the TurtleBot all over the map to create a precise 2-D map. 
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 run my_py_pkg robot_command_publishrer
```
![Screenshot from 2024-10-14 23-12-48](https://github.com/user-attachments/assets/112703c8-2de6-49a5-bca8-c99359f14e4e)

3. RVIZ for Visualization
```bash
ros2 launch nav2_bringup rviz_launch.py
```
4. Save the Map
Once the mapping is complete, save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_custom_map
```
This will create two files my_custom_map.pgm and my_custom_map.yaml

5. Run Dijkstra'a Algorithm
```bash
python3 Dijkstra.py
```
![Maze_screenshot_14 10 2024](https://github.com/user-attachments/assets/52563786-cafd-4636-b2c8-9f245052819f)

Click on any 2 points on the map and you'll get the shortest path 

6. Run the Navigation Stack
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/path/to/my_custom_map.yaml use_sim_time:=true
```
Launch Rviz and select the map topic to visualize the bot and the 2D map. Choose the same two points as before, and you will observe the bot navigating along the predicted path while efficiently avoiding obstacles.

![Screenshot from 2024-10-14 23-11-46](https://github.com/user-attachments/assets/92059459-7d7c-43dd-8a09-3bb97ee2b463)

# How the Custom Dijkstra Algorithm Work
1. Map to Grid Conversion: The environment map is converted into a 2D grid where each cell is a node.
2. Graph Representation: The grid is treated as a graph with adjacent cells connected as neighbors.
3. Shortest Path Calculation: The Dijkstra algorithm computes the optimal path from the start to the goal.
4. Path Execution: The planned path is sent to the navigation stack, and the robot follows it autonomously.

