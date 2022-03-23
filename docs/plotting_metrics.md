## qualitative plots

## quantitative plots
The simulation data can be evaluated various different metrics:
- `Success`: Is a binary classifier which determines, whether the robot reached the navigation goal within the given time frame and below the collision threshold
- `Collisions`: Measures the number of collisions of the robot with obstacles, by checking whether obstacles have been registered by the range-laser very close to the obstacle.
- `Path-length`: Measuring the number of meters taken by the roboter to reach the navigation goal. 
- `Normalized-curvature`: Is a measure to quantify the degree of trajectory changes.
- `Roughness`: A value to quantify the trajectory smotthnes
- `Angle-over-length`: An alternative measure of smoothness
- `Velocity`: The velocity of the robot [m/s]
- `Jerk`: The jerk of the robot [m/s³]
