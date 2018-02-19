## Project Description
Model Predictive Control (MPC) is an advanced method of process control which relies on dynamic model and the current measurements to predict future values of output. MPC controller can anticipate future events and take actions accordingly.

### The Model

The model used is a Kinematic model (bicycle model) neglecting the complex interactions between the tires and the road. The model states are:
- `x, y` : Car's position.
- `psi` : Car's heading direction.
- `v` : Car's velocity.
- `cte` : Cross-track error.
- `epsi` : Orientation error.

In addition to that, `Lf` is the distance between the car of mass and the front wheels and the remaining two values are the model output:

- `a` : Car's acceleration (throttle).
- `delta` : Steering angle.

The update equations are as follow:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

The objective is to find the acceleration (`a`) and the steering angle(`delta`) in the way it will minimize an objective function that is the combination of different factors:

- Square sum of `cte` and `epsi`.
- Square sum of the difference actuators to penalize a lot of actuator's actions.
- Square sum of the difference between two consecutive actuator values to penalize sharp changes..

### Timestep Length and Elapsed Duration (N & dt)

The prediction horizon T is the product of the timestep length N and elapsed duration dt. Timestep length refers to the number of timesteps in the horizon and elapsed duration is how much time elapses between each actuation.

The prediction horizon I empirically set these parameters to be N = 10 and dt = .1.

I tried different combinations of N and dt, including (N=20, dt=0.05), (N=15, dt=0.05), (N=10, dt=0.05), (N=20, dt=0.1) and so on. With higher N value, if the vehicle "overshot" the reference trajectory, it would begin to oscillate wildly and drive off the track. With lower value of N, the vehicle may drive straight off the track.

### Polynomial Fitting and MPC Preprocessing

The point are first transformed into the vehicle's coordinate system, making the first point the origin. This is done by subtracting each point from the current position of the vehicle.

Next, the orientation is also transformed to 0 so that the heading is straight forward. Each point is rotated by psi degrees.

After that the vector of points is converted to an Eigen vector so that it is ready to be an argument in the polyfit function where the points are fitted to a 3rd order polynomial. That polynomial is then evaluated using the polyeval function to calculate the cross-track error.

### Model Predictive Control with Latency

A delay of 100 ms need to be taken care of after the MPC works. When this latency was first introduced, the model oscillated about the reference trajectory and, at high speeds, drove off the track.

In order to deal with this latency, I set dt to be equal to latency to handle the actuations.
