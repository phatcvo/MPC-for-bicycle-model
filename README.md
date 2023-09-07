The MPC controller controls vehicle speed and steering base on linealized model.
This code uses [CVXPY](http://www.cvxpy.org/) as an optimization modeling tool 

### Vehicle model linearization
Vehicle model is 
$$ \dot{x} = vcos(\phi)$$
$$ \dot{y} = vsin((\phi)$$
$$ \dot{v} = a$$
$$ \dot{\phi} = \frac{vtan(\delta)}{L}$$

ODE is $$ \dot{z} =\frac{\partial }{\partial z} z = f(z, u) = A'z+B'u$$
where
$$ A' =
\begin{bmatrix}
\frac{\partial }{\partial x}vcos(\phi) & 
\frac{\partial }{\partial y}vcos(\phi) & 
\frac{\partial }{\partial v}vcos(\phi) &
\frac{\partial }{\partial \phi}vcos(\phi)\\
\frac{\partial }{\partial x}vsin(\phi) & 
\frac{\partial }{\partial y}vsin(\phi) & 
\frac{\partial }{\partial v}vsin(\phi) &
\frac{\partial }{\partial \phi}vsin(\phi)\\
\frac{\partial }{\partial x}a& 
\frac{\partial }{\partial y}a& 
\frac{\partial }{\partial v}a&
\frac{\partial }{\partial \phi}a\\
\frac{\partial }{\partial x}\frac{vtan(\delta)}{L}& 
\frac{\partial }{\partial y}\frac{vtan(\delta)}{L}& 
\frac{\partial }{\partial v}\frac{vtan(\delta)}{L}&
\frac{\partial }{\partial \phi}\frac{vtan(\delta)}{L}
\end{bmatrix}
　=
\begin{bmatrix}
0 & 0 & cos(\bar{\phi}) & -\bar{v}sin(\bar{\phi})\\
0 & 0 & sin(\bar{\phi}) & \bar{v}cos(\bar{\phi}) \\
0 & 0 & 0 & 0 \\
0 & 0 &\frac{tan(\bar{\delta})}{L} & 0 \\
\end{bmatrix}
$$

### MPC modeling
State and Input vector:
$$ z = [x, y, v,\phi]$$
$$ u = [a, \delta]$$
x: x-position; y:y-position; v:velocity; φ: yaw angle; a: accellation; δ: steering angle

The MPC cotroller minimize this cost function for path tracking:
$$J = min\ Q_f(z_{T,ref}-z_{T})^2+Q\Sigma({z_{t,ref}-z_{t}})^2+R\Sigma{u_t}^2+R_d\Sigma({u_{t+1}-u_{t}})^2$$
z_ref come from target path and speed.
subject to:
- Linearlied vehicle model
  $$z_{t+1}=Az_t+Bu+C$$
- Maximum steering speed
  $$|u_{t+1}-u_{t}|<du_{max}$$
- Maximum steering angle
  $$|u_{t}|<u_{max}$$
- Initial state
  $$z_0 = z_{0,ob}$$
- Maximum and minimum speed
  $$v_{min} < v_t < v_{max}$$
- Maximum and minimum input
  $$u_{min} < u_t < u_{max}$$
