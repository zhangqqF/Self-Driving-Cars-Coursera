# MPC

> zhangqq  
>
> Mar-16, 2023
>
> Chongqing

---


<p align=center>
<img src=./img/MPC_vehicleDynamic.png>
</p>

***v*<sub>f</sub>**   -- Velocity of front

***δ***	-- Rotate angle of front wheel (steering angle)

***φ***	-- Rotate angle of rear wheel (vehicle body angle, hesding angle)

***R***	-- Steering radius

***P***	-- Steering canter point

***L***	-- Wheel base

**(*X*<sub>f</sub>,** ***Y*<sub>f</sub>)**   -- Displacement of the rear wheel



Don't follow the derivation of the book 无人驾驶车辆模型预测控制-龚建伟, it's suck.

So, the velocity of the rear wheel ***v*<sub>r</sub>** is:
$$
v_r = \dot{X}_rcos\varphi + \dot{Y}_rsin\varphi
$$


