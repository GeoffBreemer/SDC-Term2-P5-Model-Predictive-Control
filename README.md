# Project 5: Model Predictive Control


## The Model
For this project the global kinematic model is used. It is a simplified dynamic model that ignores tire dynamics.

### State
The state tracks the position (i.e. x and y coordinates), orientation and speed of the car.

### Actuators
To control the car's state two actuators were used:

- `delta`: the steering angle, normalised and constrainted to [-1, 1]
- `a`: acceleration with positive values modelling an accelerating car, and negative values a braking car, also constrained to [1, -1].

### State update equations
The actuators change the state over time like this:

- `x position`: old x position plus speed times the cosine of the orientation times the time step
- `y position`: old y position plus speed times the sine of the orientation times the time step
- `orientation`: old orientation plus speed times the steering angle times the time step / divided by a multiplicative factor.  
- `speed`: old speed plus acceleration times the time step

Note:

- The multiplicative factor is a car specific constant. This constant (2.67) was given in the assignment for the simulated car being used. It measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate
- The time step is the number of milliseconds between the current time and the time when the previous state was determined.

### Minimising the error
To minimise the error between the reference trajectory and the car's actual path the cost function penalises the current state as follows:

- `cross track error`: the difference between the center of the road and the car's position, the goal is to make this error zero
- `orientation error`: the difference between the current orientation and the desired orientation, the goal is to make this error zero as well
- `speed error`: the difference between the current speed and the target speed. In the model used for the project the target speed is a constant 100MPH, i.e. the car will attempt to always maintain the target speed. 

The cost function also penalises actuator input:

- `actuator magnitude`: large, i.e. more extreme, actuator inputs are penalised more than smaller, i.e. smoother, ones
- `actuator change rate`: prevent sudden changes in actuator inputs between time stamps (i.e. sudden jerks of the steering wheel, fully pressing the accelerator)  

The various components that make up the cost function shown above, are each multiplied by a factor. The factors were determined through trial and error. For example, it seems that heavily penalizing steering wheel magnitude and change rate (e.g. using a high factor value of 5000), limits oscilliations and results in a smoother ride. Similarly, penalizing the accelerator magnitude and change rate also increases smoothness. However, it results in a car that accelerators (and brakes) very poorly. The result is that the car never reaches its target speed and consequently takes forever to go around the track. The factors can thus be used to find a balance between the car flying around the track and doing so in a smooth way, without crashing.

## Timestep length and frequency
Various values of `N` (number of time steps) and `delta t` (time between actuations), and thus `T` (prediction horizon in seconds) were tried. The final values used are:

- `N` is 10 time steps
- `delta t` is 0.1 second
- `T` is 1 second

It seems that higher values of `T`, i.e. looking further into the future, quickly results in wild oscillations and crashes. Small values of `delta t`, even when `T` was kept at 1 second by increasing `N` accordingly, *also* kept resulting in wild oscillations and crashes. This was a surprising result: with `T` kept constant, an increase in the number of time steps was expected to result in the car following the reference trajectory more accurately, albeit at a higher computational cost. Presumably the factors chosen to multiply the various components of the cost function as described earlier, play a role in this as well and will have to be changed as `T` changes.

Another complicating factor seemed to be that the target speed also seemed to play a role in determining `N`, `T` and `delta t`. The target speed was therefore first set and not changed going forward. Trial and error resulted in choosing the values shown above.

## Polynomial Fitting and MPC Preprocessing
Reference waypoints are converted into the car's coordinate space before a third degree polynomial is fitted.

## Model Predictive Control with Latency
Latency is handled by taking the current state and speed and projecting the x position 100ms into the future. The updated x position along with all other state elements (which are not changed) are passed to the solver. 
