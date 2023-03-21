

# MPC

> zhangqq  
> Mar-16, 2023  
> Chongqing

---


- [MPC](#mpc)
	- [Chapter 2 Kinemic and Dynamic Model of the Vehicle](#chapter-2-kinemic-and-dynamic-model-of-the-vehicle)
		- [2-1 Kinemic Model](#2-1-kinemic-model)
		- [2-2 Dynamic Model](#2-2-dynamic-model)
		- [2-3 Wheel Model](#2-3-wheel-model)
	- [Linear MPC](#linear-mpc)
	
	



Reference the book ***Model Predictive Control fo the Self-Driving Cars***.

## Chapter 2 Kinemic and Dynamic Model of the Vehicle

### 2-1 Kinemic Model


<p align=center>
<img src=./img/MPC_vehicleDynamic.png>
</p>

　　　　　　$v_f$   -- Velocity of front wheel

　　　　　　$v_r$   -- Velocity of rear wheel (vehicle velocity)

　　　　　　$\delta$	 -- Rotate angle of front wheel (steering angle)

　　　　　　$\varphi$	-- Rotate angle of rear wheel (vehicle body angle, hesding angle)

　　　　　　$R$	-- Steering radius

　　　　　　$P$	-- Steering canter point

　　　　　　$L$	-- Wheel base

　　　　　　$(X_f, Y_f)$   -- Displacement of the front wheel

　　　　　　$(X_r, Y_r)$   -- Displacement of the rear wheel (vehicle displacement)



Don't follow the derivation of the book *Model Predictive Control fo the Self-Driving Cars*, it's suck.

The four wheels vehicle is simplify to the bycicle with two wheels, so

$$
v_f\cos\delta=v_r
$$

and, $v_f\sin\delta$ is velocity of the front wheel rotate about the rear wheel with the radius of $L$. So the angular velocity of the rear wheel is:

$$
\omega=\dot\varphi=\frac{v_f\sin\delta}{L}
$$

Use $\frac{v_r}{\cos\delta}$ replace the $v_f$, so

$$
\omega=\frac{v_r\tan\delta}{L}
$$

We can also have the kinemic model easily.

$$
\begin{bmatrix}
	v_r \\
	v_r \\
	\omega
\end{bmatrix}=
\begin{bmatrix}
	\dot X_r \\
	\dot Y_r \\
	\dot\varphi
\end{bmatrix}=
\begin{bmatrix}
	\cos\varphi \\
	\sin\varphi \\
	\frac{\tan\delta}{L}
\end{bmatrix} v_r
$$


### 2-2 Dynamic Model

Ignored

### 2-3 Wheel Model

Magic Formula comes from *Pacejka*, which general for pure slip is
$$
Y(x)=D\sin\{C\arctan{[Bx-E(Bx-\arctan(Bx))]}\}
$$
　　　　　　where
$$
\begin{equation}
    \left\{
        \begin{array}{cc}	%cc refers to the align, c refers to center
            Y(x)&=&y(x)+S_v \\
            x&=&X+S_h
        \end{array}
    \right.
\end{equation}
$$
Where $Y$ is the output variable, can be longitudinal force $F_x$, side force $F_y$, or alining moment $M_z$. $B$, $C$, $D$​ refers to stiffness, shape, peak and curvature factors respectively. $S_h$ and $S_v$ refers to horizontal and vertical shift respectively.

## Linear MPC

**Question description:**

A vehicle run from the original position, track the path of $y=2$ with a longitudinal velocity of 1 m/s in sample time of 50 ms and the total time of 20 s.



Review the *Kinemic Model* before
$$
\begin{bmatrix}
	v_r \\
	v_r \\
	\omega
\end{bmatrix}=
\begin{bmatrix}
	\dot X_r \\
	\dot Y_r \\
	\dot\varphi
\end{bmatrix}=
\begin{bmatrix}
	\cos\varphi \\
	\sin\varphi \\
	\frac{\tan\delta}{L}
\end{bmatrix} v_r
$$
上式满足MPC方程
$$
\boldsymbol{\dot\xi}=f(\boldsymbol{\xi}, \boldsymbol{u}) \tag{a}
$$
令 $\boldsymbol{\xi}=[X_r, Y_r, \varphi]^T$，为状态量； $\boldsymbol{u}=[v_r, \delta]^T$ 为控制量

#### Linearization

But this is a no-linear model, and we want a linear model similar to $\boldsymbol{\dot\xi}=\boldsymbol{A}\boldsymbol{\xi}+\boldsymbol{B}\boldsymbol{u}$, so we expanse it by *Taylor Expanssion* and take to only the first order.

泰勒公式在 $x_0$处的展开：
$$
f(x)=f(x_0)+\frac{f'(x_0)}{1!}(x-x_0)+\frac{f''(x_0)}{2!}(x-x_0)^2+\cdots+\frac{f^{(n)}(x_0)}{n!}(x-x_0)^n+R_n
$$
取到第一阶，以线性化：
$$
\dot{\xi}=f(\xi{_0}, u_0)+
\frac{\partial f}{\partial \xi} \bigg|_{\begin{array}{cc}\xi=\xi_{0} \\ u=u_0 \end{array}} (\xi-\xi{_0})+
\frac{\partial f}{\partial u} \bigg|_{\begin{array}{cc}\xi=\xi_{0} \\ u=u_0 \end{array}} (u-u_0)

% github not support \begin{split}
$$
subtracted by *fromula (a)*
$$
(\dot\xi-\dot\xi_{0})+A(\xi-\xi_{0})+B(u-u_0)=0
$$
$$
\left\{
    \begin{aligned}
        A = \frac{\partial{f}}{\partial{\xi}} \bigg{|} _ {\begin{matrix} \xi=\xi_0 \\ u=u_0 \end{matrix}} \\
        B = \frac{\partial{f}}{\partial{u}} \bigg{|} _ {\begin{matrix} \xi=\xi_0 \\ u=u_0 \end{matrix}}
    \end{aligned}
\right.
% \begin{array} will cause the partials become smaller.
$$

so we can write as the linear model
$$
\boldsymbol{\dot{\tilde{\xi}}=A\tilde{\xi} + B\tilde{u}}
$$
$\tilde{i}, i = [\dot\xi, \xi, u]$ refers to
$$
\left\{
    \begin{array}{cc}
        \dot{\tilde{\xi}} = \dot{\xi} - \dot{\xi_{0}} \\
        \tilde{\xi} = \xi - \xi_{0} \\
        \tilde{u} = u - u_{0}
    \end{array}
\right.
$$
Disddddd, forward-Euler
$$
\boldsymbol {\dot{\tilde\xi}} = \frac{\boldsymbol{\tilde\xi}_{(k+1)}-\boldsymbol{\tilde\xi}_{k}} {T} \\

\therefore \\
\boldsymbol{\tilde\xi}_{(k+1)}-\boldsymbol{\tilde\xi}_{k} = T\boldsymbol{A\tilde{\xi}} + T\boldsymbol{B\tilde{u}} \\

\boldsymbol{\tilde\xi}_{(k+1)} = {(T\boldsymbol{A}+\boldsymbol{I})\boldsymbol{\tilde{\xi}}}_k + T\boldsymbol{B\tilde{u}} \\

\boldsymbol{\tilde\xi}_{(k+1)} = \boldsymbol{C\tilde{\xi}}_k + \boldsymbol{D\tilde{u}} \\
$$
Look up the partial of the matrix
$$
\boldsymbol{J}(x_1, x_2, x_3) = 
\begin{bmatrix}
    \displaystyle\frac{\partial{f_1}}{\partial{{x_1}}} & \displaystyle\frac{\partial{f_1}}{\partial{{x_2}}} & \displaystyle\frac{\partial{f_1}}{\partial{{x_3}}} \\
    \displaystyle\frac{\partial{f_2}}{\partial{{x_1}}} & \displaystyle\frac{\partial{f_2}}{\partial{{x_2}}} & \displaystyle\frac{\partial{f_2}}{\partial{{x_3}}} \\
    \displaystyle\frac{\partial{f_3}}{\partial{{x_1}}} & \displaystyle\frac{\partial{f_3}}{\partial{{x_2}}} & \displaystyle\frac{\partial{f_3}}{\partial{{x_3}}} \\
\end{bmatrix}

% bmatrix [], pmatrix (), vmatrix {}
$$
s
$$
\boldsymbol C =
\begin{bmatrix}
    \displaystyle\frac{\partial{v_r\cos{\varphi}}}{\partial{X}_r} & \displaystyle\frac{\partial{v_r\cos{\varphi}}}{\partial{Y}_r} & \displaystyle\frac{\partial{v_r\cos{\varphi}}}{\partial{(\frac{\tan{\delta}}{L})}} \\
    \displaystyle\frac{\partial{v_r\sin{\varphi}}}{\partial{X}_r} & \displaystyle\frac{\partial{v_r\sin{\varphi}}}{\partial{Y}_r} & \displaystyle\frac{\partial{v_r\sin{\varphi}}}{\partial{(\frac{\tan{\delta}}{L})}} \\
    \displaystyle\frac{\partial{v_r\varphi}}{\partial{X}_r} & \displaystyle\frac{\partial{v_r\varphi}} {\partial{Y}_r} & \displaystyle\frac{\partial{v_r\varphi}}{\partial{(\frac{\tan{\delta}}{L})}} \\
\end{bmatrix} T + \boldsymbol{I}
=
\begin{bmatrix}
    0 & 0 & -Tv_r\sin{\varphi} \\
    0 & 0 &  Tv_r\cos{\varphi} \\
    0 & 0 &  0 \\
\end{bmatrix} + 
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    0 & 0 & 1 \\
\end{bmatrix}
=
\begin{bmatrix}
    1 & 0 & -Tv_r\sin{\varphi} \\
    0 & 1 & Tv_r\cos{\varphi} \\
    0 & 0 & 1 \\
\end{bmatrix}
$$

$$
\boldsymbol D =
\begin{bmatrix}
    \displaystyle\frac{\partial{v_r\cos{\varphi}}}{\partial{v}_r} & \displaystyle\frac{\partial{v_r\cos{\varphi}}}{\partial{\delta}} \\
    \displaystyle\frac{\partial{v_r\sin{\varphi}}}{\partial{v}_r} & \displaystyle\frac{\partial{v_r\sin{\varphi}}}{\partial{\delta}} \\
    \displaystyle\frac{\partial{v_r\frac{\tan\delta}{L}}}{\partial{v}_r} & \displaystyle\frac{\partial{v_r\frac{\tan\delta}{L}}} {\partial{\delta}} \\
\end{bmatrix} T
=
\begin{bmatrix}
    T\cos\varphi & 0 \\
    T\sin\varphi & 0 \\
    \frac{T\tan\delta}{L} & \frac{Tv_r}{L\cos^2{\delta}}\\
\end{bmatrix}
$$

![](http://latex.codecogs.com/gif.latex?\\frac{\\partial J}{\\partial \\theta_k^{(j)}}=\\sum_{i:r(i,j)=1}{\\big((\\theta^{(j)})^Tx^{(i)}-y^{(i,j)}\\big)x_k^{(i)}}+\\lambda \\xtheta_k^{(j)})

