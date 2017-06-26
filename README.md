# Model Predictive Control Project
[CarND Term 2 Model Predictive Control (MPC) Project]
(https://github.com/udacity/CarND-MPC-Project)

---

## Overview
In this project, the aim is to implement a Model Predictive Controller (MPC) in C++. The controller is responsible for calculating the steering angle and the throttle value.

## Sample project video

[![PID Controller](http://img.youtube.com/vi/HMw8wZltd88/0.jpg)](http://www.youtube.com/watch?v=HMw8wZltd88)


## The Project
In order to implement MPC, the MPC class and the main.cpp file were modified.


### Way Points
In the main.cpp, when new data received from simulator, the points data were converted to vehicle coordinates, then transformed to vectors. The transformed points data was fit in to a third order polynomial.


### N and dt
The N parameter was selected as 10. Increasing N caused longer solution times and did not provide better results. Decreasing N resulted in poor results.
The dt was selected as 0.1 in the beginning. Then, to activate actuators in advance it was increased to 0.17. This enabled completing the track with higher speeds.

### Target Velocity
In order to have higher speed at straight sections and lower speed at sections with curve, the reference velocity was calculated as a function of the cte value. Other alternative tested were calculating the reference velocity as a function of the slope of the polynomial at the cars location. Using cte had better results.

### Cost Function
To drive faster, the costs from actuations are omitted. Costs from cte, epsi, and velocity were used.

To penalize higher cte, two addition cte_limits used. One of them was 2.0 and the other was 3.5. Two dummy variables introduced for every cte value. Their upperbounds were set to 2.0, and 3.5 respectively, and their lowerbounds were set to -2.0, and -3.5. Since the solver tries to minimize the total cost, if the cte value was between -2.0 and 2.0, the first dummy cte variable would be equal to the cte value, and the additional cost for that cte limit would be zero. On the other hand, if cte value is greater than 2.0 or less than -2.0, the dummy cte variable would be set to 2.0 or -2.0. As a result, the difference between the absolute value of the cte value and 2.0 would be squared and added to total cost. The cte limit with between -3.5 and 3.5, was added to the total cost in the same way.

```
fg[0] += ((10 - t) / 5.5) * c_cte_l * CppAD::pow(vars[cte_start+t] - vars[cte_cor_start+t],2);
fg[0] += ((10 - t) / 5.5) * c_cte_l2 * CppAD::pow(vars[cte_start+t] - vars[cte_cor_start2+t],2);
```

In order to increase the importance of the actuations in the first steps, a multiplier added to the cte limit costs. The cost of the first step was multiplied by 10, the cost of the second step was multiplied by 9, and so on. Then these costs were divided by 5.5, which is the average value of the multipliers for ten steps (10, 9, 8, 7, 6, 5, 4, 3, 2, 1). This multiplier enabled driving at higher speeds.


The cost from the cte value, epsi value and the difference between reference velocity and the current velocity were also added to the total cost.

```
      fg[0] += c_cte * CppAD::pow(vars[cte_start+t],2);
      fg[0] += c_epsi * CppAD::pow(vars[epsi_start+t],2);
      fg[0] += c_v * CppAD::pow(vars[v_start+t] - ref_v,2);
```

### Delay
In order to supply appropriate values for actuations, the actuations for the first step were set to zero, and the actuations for the second step were sent to the simulator. Since was dt = 0.1 and delay was 0.1 seconds, the actuations of the second step were able to drive the track. However, the car started turning as the curve began or after the curve began. In order to make the car start turning in advance, the dt value was increased to 0.17. This enabled the car to go faster.


## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* Fortran Compiler
* [Ipopt](https://projects.coin-or.org/Ipopt)
* [CppAD](https://www.coin-or.org/CppAD/)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page).

## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
