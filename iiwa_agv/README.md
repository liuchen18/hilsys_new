# Introduction

This package is for control of iiwa agv to track end-effector trajectories.

# Components
1. iiwa agv model files (urdf, transmission, gazebo)
2. iiwa agv control files (joint_trajectory_controller, velocity_controller)

# Current tasks
- [x] Construct the urdf files for iiwa agv
- [x] Read the trajectory from `txt` files
- [ ] Resolve the problems of sending the traj pts at the same time
- [ ] Create the velocity controllers for multiple joints