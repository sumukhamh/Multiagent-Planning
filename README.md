# Multiagent-Planning
## Overview
The repository is a ROS Package for multiagent_planning. The repository has multiple 2 types of nodes:
1. Agent
2. Planner
Please find the relevant documentation with respect to each node in the respective .cpp files.

## Demo
1. Clone the repository into workspace.
2. Compile
3. In new terminal `roslaunch multiagent_planning agent.launch serial_id:=agent_1 x:=2 y:=0 yaw:=0`
4. In new terminal `roslaunch multiagent_planning agent.launch serial_id:=agent_2 x:=0 y:=3 yaw:=0`
5. Launch other agent nodes if needed
6. In new terminal 'rosrun multiagent_planning planner'
7. In new terminal 'rosservice call /agent_1/update_goal "pose: x:2 y:5 yaw:0.0"'
8. Path shown on planner terminal.
9. In new terminal 'rosservice call /agent_2/update_goal "pose: x:6 y:3 yaw:0.0"'
10. Path shown on planner terminal.
