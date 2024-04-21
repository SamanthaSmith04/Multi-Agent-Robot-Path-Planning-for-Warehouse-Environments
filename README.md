# Multi Agent Robot Path Planning for Warehouse Environments
### ECE/MECHE 5463 Final Project 
### Authors: Samantha Smith, Anutam Srinivasan, Izaiah Wiley

Warehouse environments can boost efficiency by using several robots to complete simple tasks such as picking up and moving items across the warehouse. However, multiple robot (multi-agent) path planning, requires additional consideration to minimize collisions. To test multi-agent path planning in the warehouse environment, two maps were created in MATLAB to emulate a warehouse. Then, Flow Annotated Replanning (FAR), and dynamic RRT algorithms were used for path planning. FAR creates a directed graph to represent the environment by only allowing movement in one direction per row and column. It then uses A* to path plan, and a centralized controller in real-time to avoid collisions. Dynamic RRT, utilizes traditional RRT sampling methods to create a path, then uses a decentralized setup in real-time such that each robot replans if an obstacle is detected through lidar. In real-time, motion planning is used to simulate the controls required for a differential-drive robot to follow the planned path. To evaluate these approaches, we consider the travel time, path length, and proximity to other robots. The results suggest, for identical environments that FAR (with a centralized control) provides larger proximity between robots when compared to RRT. However, we find no clear comparison can be made regarding path length and travel time since each algorithms performance in these metrics is situationally dependent.


## FAR Replanning Implementation:

![ScreenRecording2024-04-21at5 10 38PM-ezgif com-video-to-gif-converter](https://github.com/SamanthaSmith04/Multi-Agent-Robot-Path-Planning-for-Warehouse-Environments/assets/82625799/bb1950e6-2dce-4587-98c7-38a65501b048)

## Dynamic RRT Replanning Implementation:

![REPLANNINGGRAPH2-ezgif com-video-to-gif-converter](https://github.com/SamanthaSmith04/Multi-Agent-Robot-Path-Planning-for-Warehouse-Environments/assets/82625799/ff88708f-a60e-408d-a9cf-6ef34a891120)


## Dependencies
- Mobile Robotics Simulation Toolbox
- Robotics System Toolbox
- Navigation Toolbox
- Optimization Toolbox
