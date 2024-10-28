# Ramp Trajectory Generator

**Author:** BeratComputer

This project provides a simple way to create ramp trajectory plans for any motion system. The variables in the ramp trajectory have no fixed units. The position and its derivatives are defined as `x(t)`, `v(t)`, and `a(t)`, where:

- `x(t)` = Position
- `v(t)` = Velocity (1st derivative of position)
- `a(t)` = Acceleration (2nd derivative of position)

All values are unit-independent, meaning the initial unit of position determines the units of velocity and acceleration. The time unit is in **seconds**.

For example, if you want to calculate the motion of 100 meters over 3 seconds, your position unit becomes meters (`x` in meters), velocity unit becomes meters per second (`v` in m/s), and acceleration unit becomes meters per second squared (`a` in m/s²).

---

## 1. Introduction

This code offers two main functions:

- **`createTrajectory`**: Generates a trajectory based on user input and provides details about the ramp profile.
- **`goWithTrajectory`**: Continuously calculates the expected position (and optionally velocity) of the system at any given moment during the trajectory. This function should be called regularly, with the returned value being used as the current setpoint.

Both functions are designed to work together, and a block diagram illustrating their usage is provided below.

### 1.1 Block Diagram of Function Usage
![Block Diagram](4.0) 
(it is not available yet!)
---

## 2. Trajectory Parameters

A ramp trajectory is defined by the following key parameters:

1. **Position (`x`)**: The distance to be covered.
2. **Velocity (`v`)**: The rate of change of position (can have a maximum value).
3. **Acceleration (`a`)**: The rate of change of velocity.
4. **Time (`T`)**: The total time duration for the trajectory.

*Default Parameters of Ramp Trajectory*
![a Ramp Trajectory default parameters.](helping_items\ramp_trajectory_parameters.jpg)

---

## 3. The `createTrajectory` Function

This function generates the ramp profile based on the given input values. Depending on which values are provided, it constructs the trajectory by evaluating various conditions. If a maximum velocity (`Vmax`) is not provided, the system assumes a velocity limit, as practical motion systems always have a speed limit. Therefore, there is no need to check for the absence of `Vmax`.

The only necessary input is the target position, but the user may provide additional inputs like acceleration (`a`) and time (`T`). This creates four possible input combinations.

![CreateTrajectory Inputs and Outputs](helping_items\createTrajectory_input_output.jpg)
The outputs describing the motion plan created by the `createTrajectory` function, based on the given inputs, are shown in the figure above.

### 3.1 Flowchart of `createTrajectory`
![CreateTrajectory Flowchart](helping_items\createTrajectory_flowchart_v1.png)

### 3.2 Important Calculations

#### Discriminant Calculation

The `createTrajectory` function uses a discriminant formula to solve for `t1` (time for reaching maximum velocity). This discriminant depends on the position (`x`), time (`T`), and acceleration (`a`). The formula for calculating `t1` is derived from these variables.

![Discriminant Formula](6.0 ) *'is not available yet'*

Once `t1` is calculated, the trajectory can be generated. If the maximum velocity is exceeded, the trajectory cannot be applied. Additionally, if the discriminant is negative, it means the given time and acceleration are insufficient for reaching the target position.

The scenarios where the discriminant is used are explained in detail below.

---

## 4. The `goWithTrajectory` Function

The `goWithTrajectory` function returns the current position (and optionally the velocity) at the time it's called. This should be used as the system's setpoint. By regularly calling this function, the system will follow the planned trajectory.

---

## 5. Summary

This project provides a simple yet flexible way to generate and follow ramp trajectories in motion control systems. By understanding how the `createTrajectory` and `goWithTrajectory` functions work together, you can create a smooth and effective motion plan for your system. 

For further explanation on trajectory calculations and flow, please refer to the flowcharts and formulas provided in the project.

--- 

*Version 1.0*

---

