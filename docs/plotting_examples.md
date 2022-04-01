# Introduction
The performance of a robot can be measured by a number of different metrics. The appropriate visualization depends on the type of parameters selected. Within the bounds of readability, the following selection of parameters is possible:
- [One variable Visualization](#one-variable-visualization)
- [Two variable Visualization](#two-variable-visualization)
- [Three variable Visualization](#three-variable-visualization)

The following performance parameter are being calculated by the arena-evaluation script:
#### Qualitative Plots
These plots show the robot trajectories using the data from the scenario file, the occupancy map and the recorded robot Odometry. Note that these plots can only be created in *scenario* mode.

<p align="center">
  <img src="/docs/imgs/qualitative-plots.png">
</p>

#### Quantitative Plots
The simulation data can be evaluated various different metrics:
- `Success`: Is a binary classifier which determines, whether the robot reached the navigation goal within the given time frame and below the collision threshold
- `Collisions`: Measures the number of collisions of the robot with obstacles, by checking whether obstacles have been registered by the range-laser very close to the obstacle.
- `Path-length`: Measuring the number of meters taken by the roboter to reach the navigation goal. 
- `Normalized-curvature`: Is a measure to quantify the degree of trajectory changes.
- `Roughness`: A value to quantify the trajectory smoothness
- `Angle-over-length`: An alternative measure of smoothness
- `Velocity`: The velocity of the robot [m/s]
- `Jerk`: The jerk of the robot [m/s³]

The following table gives you an insight into the variable type of the parameter, which is important for choosing the right representation.

| **Categorical** | **Continuous**  | **Binary** |
| :---------------------------- | :---------------------- | :------------------- |
| - Robot  <br> - World  <br> - Obs(tacles)  <br> - Planner  <br> - done\_reason        | - Time (to goal)  <br> - (collision) (actually discrete)  <br> - curvature  <br> - normalized\_curvature  <br> - roughness  <br> - jerk <br> - acc  <br> - vel  <br> - path\_length <br> - angle\_over\_length  <br> - max\_curvature | - success |

---
### One variable Visualization
---
### Two variable Visualization
---
### Three variable Visualization