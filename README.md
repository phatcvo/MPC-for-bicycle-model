The MPC controller controls vehicle speed and steering base on linealized model.
This code uses [CVXPY](http://www.cvxpy.org/) as an optimization modeling tool 

## Vehicle model linearization
Vehicle model is 

$$ \dot{x} = vcos(\phi)$$

$$ \dot{y} = vsin((\phi)$$

$$ \dot{v} = a$$

$$ \dot{\phi} = \frac{vtan(\delta)}{L}$$

ODE is 

$$ \dot{z} =\frac{\partial }{\partial z} z = f(z, u) = A'z+B'u$$

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

$$
B' =
\begin{bmatrix}
\frac{\partial }{\partial a}vcos(\phi) &
\frac{\partial }{\partial \delta}vcos(\phi)\\
\frac{\partial }{\partial a}vsin(\phi) &
\frac{\partial }{\partial \delta}vsin(\phi)\\
\frac{\partial }{\partial a}a &
\frac{\partial }{\partial \delta}a\\
\frac{\partial }{\partial a}\frac{vtan(\delta)}{L} &
\frac{\partial }{\partial \delta}\frac{vtan(\delta)}{L}\\
\end{bmatrix}
　=
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1 & 0 \\
0 & \frac{\bar{v}}{Lcos^2(\bar{\delta})} \\
\end{bmatrix}
$$

You can get a discrete-time mode with Forward Euler Discretization with sampling time dt.

$$z_{k+1} = z_k+f(z_k,u_k)dt$$

Using first degree Tayer expantion around zbar and ubar
$$z_{k+1} = z_k+(f(\bar{z},\bar{u})+A'z_k+B'u_k-A'\bar{z}-B'\bar{u})dt$$

$$z_{k+1} = (I + dtA')z_k+(dtB')u_k + (f(\bar{z},\bar{u})-A'\bar{z}-B'\bar{u})dt$$

So, 

$$z_{k+1} = Az_k+Bu_k +C$$

where

$$A = (I + dt A') =
\begin{bmatrix} 
1 & 0 & cos(\bar{\phi})dt & -\bar{v}sin(\bar{\phi})dt\\
0 & 1 & sin(\bar{\phi})dt & \bar{v}cos(\bar{\phi})dt \\
0 & 0 & 1 & 0 \\
0 & 0 &\frac{tan(\bar{\delta})}{L}dt & 1 \\
\end{bmatrix}$$

$$B = dt B' =
\begin{bmatrix} 
0 & 0 \\
0 & 0 \\
dt & 0 \\
0 & \frac{\bar{v}}{Lcos^2(\bar{\delta})}dt \\
\end{bmatrix}$$


$$ C = (f(\bar{z},\bar{u})-A'\bar{z}-B'\bar{u})dt\\
B'\bar{u})dt\\
= dt(\begin{bmatrix} 
\bar{v}cos(\bar{\phi})\\
\bar{v}sin(\bar{\phi}) \\
\bar{a}\\
\frac{\bar{v}tan(\bar{\delta})}{L}\\
\end{bmatrix} - 
\begin{bmatrix} 
\bar{v}cos(\bar{\phi})-\bar{v}sin(\bar{\phi})\bar{\phi}\\
\bar{v}sin(\bar{\phi})+\bar{v}cos(\bar{\phi})\bar{\phi}\\
0\\
\frac{\bar{v}tan(\bar{\delta})}{L}\\
\end{bmatrix} -
\begin{bmatrix} 0\\ 0\\ \bar{a}\\
\frac{\bar{v}\bar{\delta}}{Lcos^2(\bar{\delta})}\\
\end{bmatrix}) =
\begin{bmatrix} 
\bar{v}sin(\bar{\phi})\bar{\phi}dt\\
-\bar{v}cos(\bar{\phi})\bar{\phi}dt\\
0\\
-\frac{\bar{v}\bar{\delta}}{Lcos^2(\bar{\delta})}dt\\
\end{bmatrix}
$$

## MPC modeling
State and Input vector:

$$ z = [x, y, v,\phi] $$

$$u = [a, \delta]$$

x: x-position; y:y-position; v:velocity; φ: yaw angle; a: accellation; δ: steering angle

The MPC cotroller minimize this cost function for path tracking:

$$J = min\ Q_f(z_{T,ref}-z_{T})^2+Q\Sigma({z_{t,ref}-z_{t}})^2+R\Sigma{u_t}^2+R_d\Sigma({u_{t+1}-u_{t}})^2$$

z_ref come from target path and speed.
subject to:

 $$z_{t+1}=Az_t+Bu+C$$
  
 $$z_0 = z_{0,ob}$$
  
 $$v_{min} < v_t < v_{max}$$

 $$u_{min} < u_t < u_{max}$$


 $$|u_{t+1}-u_{t}| < du_{max}$$
 
 $$|u_{t}| < u_{max}$$
