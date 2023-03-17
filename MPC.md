

# MPC

> zhangqq  
>
> Mar-16, 2023
>
> Chongqing

---



[toc]

Reference the book ***Model Predictive Control fo the Self-Driving Cars***.

## Chapter 4 Kinemic and Dynamic Model of the Vehicle

---


<p align=center>
<img src=./img/MPC_vehicleDynamic.png>
</p>

​			$v_f$   -- Velocity of front wheel

​			$v_r$   -- Velocity of rear wheel (vehicle velocity)

​			$\delta$	 -- Rotate angle of front wheel (steering angle)

​			$\varphi$	-- Rotate angle of rear wheel (vehicle body angle, hesding angle)

​			$R$	-- Steering radius

​			$P$	-- Steering canter point

​			$L$	-- Wheel base

​			$(X_f, Y_f)$   -- Displacement of the front wheel

​			$(X_r, Y_r)$   -- Displacement of the rear wheel (vehicle displacement)



Don't follow the derivation of the book *Model Predictive Control fo the Self-Driving Cars*, it's suck.

The four wheels vehicle is simplify to the bycicle with two wheels, so that:
$$
v_fcos\delta=v_r
$$
and, $v_fsin\delta$ is velocity of the front wheel rotate about the rear wheel with the radius of $L$. So the angular velocity of the rear wheel is:
$$
\omega=\dot\varphi=v_fsin\delta
$$
\omega$

So, the velocity of the rear wheel ***v*<sub>r</sub>** is:
$$
v_r = \dot{X}_rcos\varphi + \dot{Y}_rsin\varphi
$$

