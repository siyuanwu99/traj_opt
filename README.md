# traj_opt

![GitHub top language](https://img.shields.io/github/languages/top/edmundwsy/traj_opt) ![GitHub last commit](https://img.shields.io/github/last-commit/edmundwsy/traj_opt)

A ROS package for trajectory optimization in  $\mathbb{R}^3$. Developing...


## Quick Start

1. Clone this repo, and build
  ```
  catkin build
  ```
2. Launch the nodes for visualization
  ```shell
  roslaunch traj_opt try_bezier.launch
  ```
3. use 2D Nav tool to send a goal point, it will randomly generates safety corridors and samples times to these corridors

## TODO
- [x] Polynomial + MiniSnap
- [x] Bezier Curve
- [ ] Bspline
- [ ] MINVO
- [ ] MINCO

## Thanks

- [Btraj](https://github.com/HKUST-Aerial-Robotics/Btraj/)
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
- [EDG-TEAM](https://github.com/ZJU-FAST-Lab/EDG-TEAM)
- [MADER](https://github.com/mit-acl/mader)
