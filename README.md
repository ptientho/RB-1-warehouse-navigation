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

```git clone -b simulation --recurse-submodules https://github.com/ptientho/RB-1-warehouse-navigation.git```

```colcon build```

**Project source codes**

Inside ROS2 workspace runs,

```git clone -b main --recurse-submodules https://github.com/ptientho/RB-1-warehouse-navigation.git```

```colcon build```

**Web application**

Inside web workspace runs,

```git clone -b webapp --recurse-submodules https://github.com/ptientho/RB-1-warehouse-navigation.git```

# How to run the application

Once you already downloaded the above docker image and RB-1 robot is connected, run the following command

```docker exec -it rb1_webapp /bin/bash -c 

