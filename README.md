# Autonomous Shelf Detection in a Mobile Robot using the Behavior Tree

This project is to develop a complex behavior based on multiple simpler behaviors using the behavior tree.
The behavior consists of 

1. detect a shelf anywhere in the warehouse
2. get underneath the shelf and use the elevator to pick it up
3. place the shelf at a desired location

Thanks to the [BT.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/master) and [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) libraries, the complex behavior can be achieved.
This project is based on ROS2 Humble middleware.

# How to compile

**Robot Simulation**

Inside simuklation workspace runs,

```
git clone -b simulation --recurse-submodules https://github.com/ptientho/RB-1-warehouse-navigation.git
colcon build
```

**Project source codes**

Inside ROS2 workspace runs,

```
git clone -b main --recurse-submodules https://github.com/ptientho/RB-1-warehouse-navigation.git
colcon build
```

**Web application**

Inside web workspace runs,

```git clone -b webapp --recurse-submodules https://github.com/ptientho/RB-1-warehouse-navigation.git```

# How to run the application (simulation version)

**Gazebo Simulation**

```
source ~/your-simulation-workspace/install/setup.bash
ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml
```

**Navigation Server**

```
ros2 launch path_planner_server navigation.launch.py
```

**Rviz**

```
ros2 launch path_planner_server rviz.launch.py
```

**Shelf Servers**

```
ros2 launch rb1_autonomy shelf_servers.launch.py
```

**Autonomy Server**

```
ros2 launch rb1_autonomy autonomy.launch.py
```

**Web Application**

```
cd ~/your-webpage-workspace/RB-1-warehouse-navigation/rb1_webapp/
python3 -m http.server 7000
```

In a new terminal, run

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

# How to run the application (real robot)

Make sure to connect the real robot.

**Navigation Server**

```
ros2 launch path_planner_server navigation_real.launch.py
```

**Rviz**

```
ros2 launch path_planner_server rviz.launch.py
```

**Shelf Servers**

```
ros2 launch rb1_autonomy shelf_servers_real.launch.py
```

**Autonomy Server**

```
ros2 launch rb1_autonomy autonomy_real.launch.py
```

**Web Application**

```
cd ~/your-webpage-workspace/RB-1-warehouse-navigation/rb1_webapp/
python3 -m http.server 7000
```

In a new terminal, run

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

