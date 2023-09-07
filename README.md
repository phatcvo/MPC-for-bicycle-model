This is a path tracking simulation using model predictive control (MPC).

The MPC controller controls vehicle speed and steering base on linealized model.

This code uses cvxpy as an optimization modeling tool.

- [Welcome to CVXPY 1\.0 — CVXPY 1\.0\.6 documentation](http://www.cvxpy.org/)

### MPC modeling

State vector is:

$$ z = [x, y, v,\phi]$$

x: x-position, y:y-position, v:velocity, φ: yaw angle

Input vector is:

$$ u = [a, \delta]$$

a: accellation, δ: steering angle
