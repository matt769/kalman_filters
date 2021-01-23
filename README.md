The aim in this repository is to code up some different Kalman filters (e.g. standard, extended, unscented) and compare them.  
Also to try coding them in different ways and see how it affects various characteristics, mainly usability.

For the comparison/evaluation side of things it might have been easier to do this in a different language e.g. Python for easier access/integration with plotting and similar tools. 

### kf0
Fully specified within KalmanFilter class.


### kf1
KalmanFilter class contains a 'state' (mean and covariance). It contains no information about how the system evolves through prediction or correction, but just applies the process or measurement functions to its state. 
If multiple measurement types are available, they can be used by passing the appropriate measurement function, value and noise to the `Update` function. 
For convenience, the process and measurement models are converted to matrices - if the functions are non-linear this will be a problem (I think beyond it being a problem for the standard KF anyway).

### kf2
Like `kf1` but is given the process and measurement models upon construction. 
For convenience/efficiency, we convert these functions into matrix form and store them.
The `Update` function no longer supports multiple measurement types.