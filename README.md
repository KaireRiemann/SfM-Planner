# SfM-Planner

Independent catkin workspace for the state2state corridor click-to-goal pipeline
(fast LBFGS + warm start included).

This workspace is a **sibling** of `real_planner`, not a subdirectory of it.

```
~/ros1_ws/
  real_planner/     # General-Planner workspace (exploration etc.)
  SfM-Planner/      # this workspace (click cave state2state only)
```

Do **not** put SfM-Planner under `real_planner/src`, or ROS will report duplicate
packages (`map_manager`, `quadrotor_msgs`, `marsim_render`, ...).

## Build / run (use a clean shell)

```bash
cd ~/ros1_ws/SfM-Planner
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch sfm_planner click_cave.launch
```

For General-Planner exploration, use only `real_planner` / `General-Planner`
`devel/setup.bash` — do not overlay both workspaces in the same shell.
