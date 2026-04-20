# AutonomousNavigation

TurtleBot3 Burger autonomous navigation project - FIB UPC Robotics

## Local development

- For local simulator setup and daily commands, see `LOCAL_SIMULATION_SETUP.txt`.
- For robot + university-lab workflow, see `RUN_ROBOT_STEPS.txt`.
- For overall workflow, map generation and readiness criteria, see `PROJECT_WORKFLOW.txt`.
- For the real-session checklist, see `IN_PERSON_CHECKLIST.txt`.
- To avoid `conda` breaking ROS builds on this machine, start from the repo root with:

```bash
source source_local_env.sh
```

## Quick local simulator check

```bash
cd /home/miguel/Escritorio/ROB/project
./start_local_validation_gui.sh
```

In another terminal:

```bash
cd /home/miguel/Escritorio/ROB/project
source source_local_env.sh
ros2 run turtlebot3_teleop teleop_keyboard
```

## Headless simulator

For CLI-only testing without Gazebo GUI:

```bash
cd /home/miguel/Escritorio/ROB/project
source source_local_env.sh
source install/setup.bash
ros2 launch rob_project tb3_sim_headless.launch.py
```
