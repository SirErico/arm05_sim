# arm05_sim


## 1. Start gazebo with aruco detection
```bash
ros2 launch arm05_sim world.launch.py
```

## 2. Start nav2 stack
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py   use_sim_time:=true   map:=/arm_ws/src/arm05_sim/map/map.yaml   params_file:=/arm_ws/src/turtlebot3/turtlebot3_navigation2/param/waffle.yaml
```

## 3. Start aruco search action server
```bash
ros2 run arm05_sim aruco_search_action_server.py
```

## 4. Send goal to aruco search action server
```bash
ros2 action send_goal /search_aruco_markers arm05_sim/action/SearchAruco "{start_search: true}" --feedback
```