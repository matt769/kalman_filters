The aim in this repository is to code up some different Kalman filters (e.g. standard, extended, unscented) and compare them.  
Also to try coding them in different ways and see how it affects various characteristics, mainly usability.

For the comparison/evaluation side of things it would probably have been easier to do this in a different language e.g. Python for easier access/integration with plotting tools.  
For this reason, I'm going to stop work on this for now and probably move on to a Python version.  

There are a few different version of the standard Kalman filter implemented, the one I prefer is that located in the `kf4` namespace.  
`main.cpp` contains (very simple) examples of running all the filters, but as a brief description...

1. Implement a 'system' which contains
 - a process model to evolve the state
 - a measurement model to calculate the predicted measurement based on the current state
 - follow the layout as shown in `systems.h`

2. Use the filter!  
Assuming the system you created was called `MySystem` in the `systems` namespace
For a standard KF
```c++
kf4::KalmanFilter<systems::MySystem> kf;
kf_4.Predict(Q); // where Q is the process noise
kf_4.Update(z, R); // where z is the measurement, and R is the process measurement noise
```

For an EKF and UKF it's basically the same
```c++
ekf1::ExtendedKalmanFilter<systems::MySystem> ekf;
ekf.Predict(Q);
ekf.Update(z, R);
```
```c++
ukf1::UnscentedKalmanFilter<systems::MySystem> ukf;
ukf.Predict(Q);
ukf.Update(z, R);
```


### Limitations
I didn't want to get caught up making these too 'real' since initially my goal was just to play around. If these were to be used for real, they would require some changes. 
 - Mainly I think the addition of a more concrete concept of time (currently it's just a fixed timestep).
 - Add control into process model
 - Consider how to support multiple measurement types (the current structure in most of these examples doesn't really support it that cleanly)

### TODO
Add a bit more of an explanation of each matrix and each step in the filter (particularly for the UKF)
