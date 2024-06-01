# Safe-Returning-FT

This repository contains the implementation of `Safe-Returning-FaSTrack` introduced in the paper [Safe Returning FaSTrack with Robust Control Lyapunov-Value Functions](https://arxiv.org/pdf/2404.02472) by ZhengGong*, Boyang Li* and Sylvia Herbert.

SR-F is designed to reject unexpected disturbances during navigation in a priori unknown environments, while accelerating the process. It merges concepts from 1) [Robust Control Lyapunov-Value Functions (R-CLVF)](https://arxiv.org/pdf/2403.03455), and 2) [Fast and Safe Tracking (FaSTrack) framework](https://arxiv.org/pdf/2102.07039).

Two SR-F demos in the paper are included:
1) 8D quadrotor model
tracking a 2D integrator planner model with the A* planner,

2) 10D near-hover quadrotor tracking a 3D integrator
planner model with the RRT planner. 

## Requirements

- `hj_reachability`: https://github.com/HJReachability/helperOC
- `Control-Lyapunov-Value-Functions`: https://github.com/ZG0327/Control-Lyapunov-Value-Functions
