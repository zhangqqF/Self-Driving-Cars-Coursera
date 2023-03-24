

# MPC - Model Predictive Control

> zhangqq  
> Mar-16, 2023  
> Chongqing

---



- [MPC - Model Predictive Control](#mpc---model-predictive-control)
	- [车辆运动方程](#车辆运动方程)
		- [线性化](#线性化)
		- [离散化](#离散化)
		- [预测](#预测)
		- [优化](#优化)
- [\\](#)
- [\&\\min\[(E^T + 2E\\theta U + (\\theta U)^T)(QE+Q\\theta U) + U^TRU\]\\](#minet--2etheta-u--theta-utqeqtheta-u--utru)
	
		- [反馈控制](#反馈控制)
	
	
	

## 车辆运动方程


<p align=center>
<img src=./img/MPC_vehicleDynamic.png>
</p>

　　　　　　$v_f$   -- 前轴速度

　　　　　　$v_r$   -- 车速（后轴速度为车速）

　　　　　　$\delta$	 -- 转向角（前轴角度）

　　　　　　$\varphi$	-- 航向角（后轴角度）

　　　　　　$R$	-- 转向半径

　　　　　　$P$	-- 转向圆心

　　　　　　$L$	-- 轴距

　　　　　　$(X_f, Y_f)$   -- 前轴位移

　　　　　　$(X_r, Y_r)$   -- 后轴位移



将4轮模型简化为两轮来分析。

由几何关系可得到：
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
\end{bmatrix} v_r \tag{1}
$$

其中$\omega$为后轴角速度，前轮速度的切向分量$v_f\sin\delta$使前轮以后轮为中心，轴距$L$为半径旋转，同时带动后轮旋转。

上式正好满足MPC方程：
$$
\boldsymbol{\dot\xi}=f(\boldsymbol{\xi}, \boldsymbol{u}) \tag{2}
$$
令 $\boldsymbol{\xi}=[X_r, Y_r, \varphi]^T$，为状态量； $\boldsymbol{u}=[v_r, \delta]^T$ 为控制量。

### 线性化

上述方程是非线性方程，需将其转为形似$\boldsymbol{{\dot\xi}={A}{\xi}+{B}{u}}$的线性方程，将式（2）在$(\xi_0, u_0)$处泰勒展开并忽略二阶及后面所有项数，得：
$$
\boldsymbol{\dot\xi}=f(\boldsymbol\xi{_0}, \boldsymbol u_0)+
\frac{\partial \boldsymbol f}{\partial \boldsymbol\xi} \bigg|_{\begin{array}{cc}\boldsymbol{\xi=\xi}_{0} \\ \boldsymbol{u=u}_0 \end{array}} (\boldsymbol{\xi-\xi}_{0})+
\frac{\partial \boldsymbol f}{\partial \boldsymbol u} \bigg|_{\begin{array}{cc}\boldsymbol{\xi=\xi}_{0} \\ \boldsymbol{u=u}_0 \end{array}} (\boldsymbol{u-u}_0) \tag{3}

% github not support \begin{split}
$$
令$\boldsymbol A=\frac{\partial{\boldsymbol f}}{\partial\boldsymbol\xi}\bigg{|}_{\begin{matrix}\boldsymbol{\xi=\xi}_0 \\ \boldsymbol{u=u}_0\end{matrix}}$，$\boldsymbol B=\frac{\partial{\boldsymbol f}}{\partial\boldsymbol{u}}\bigg{|}_{\begin{matrix}\boldsymbol{\xi=\xi}_0 \\ \boldsymbol{u=u}_0\end{matrix}}$，再将式（2）减式（3），得：
$$
(\boldsymbol{\dot\xi}-\boldsymbol{\dot\xi}_{0})+\boldsymbol A(\boldsymbol\xi-\boldsymbol\xi_{0})+\boldsymbol B(\boldsymbol u-\boldsymbol u_0)=0
$$
由上式可看出$(\boldsymbol{\dot\xi-\dot\xi}_{0})$、$(\boldsymbol{\xi-\xi}_{0})$、$(\boldsymbol{u-u}_0)$分别为$\boldsymbol{\dot\xi}$、$\boldsymbol\xi$、$\boldsymbol u$相对于$\boldsymbol{\dot\xi}_0$、$\boldsymbol\xi_0$、$\boldsymbol u_0$的误差。实际上，$\boldsymbol{\dot\xi}_0$、$\boldsymbol\xi_0$、$\boldsymbol u_0$就是参考点，MPC控制的目标就是向参考点对齐。于是就得到了误差的线性化模型：
$$
\boldsymbol{\dot{\tilde{\xi}}=A\tilde{\xi} + B\tilde{u}} \tag{4}
$$
$\boldsymbol{\tilde i}$表示误差，其中$\boldsymbol{\tilde{\dot\xi}=\dot\xi-\dot\xi}_0$、$\boldsymbol{\tilde\xi=\xi-\xi}_0$、$\boldsymbol{\tilde u=u-u}_0$。

### 离散化

采用欧拉前向离散化，令$\boldsymbol {\dot{\tilde\xi}} = \frac{\boldsymbol{\tilde\xi}_{(k+1)}-\boldsymbol{\tilde\xi}_{k}} {T}$，$T$为采样时间，于是：
$$
\boldsymbol{\tilde\xi}_{(k+1)} = {(T\boldsymbol{A}+\boldsymbol{I})\boldsymbol{\tilde{\xi}}}_k + T\boldsymbol{B\tilde{u}}
$$
令$\boldsymbol{C}=T\boldsymbol{A}+\boldsymbol{I}$，$\boldsymbol{D}=T\boldsymbol{B}$，则：
$$
\boldsymbol{\tilde\xi}_{(k+1)} = {\boldsymbol{C}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{D\tilde{u}} \tag{5}
$$

代入运动方程得（$\boldsymbol{u}$没有参考值，无需减去$\boldsymbol u_0$）：
$$
\tilde{\boldsymbol\xi}_k=
\begin{bmatrix}
	X-X_0 \\
	Y-Y_0 \\
	\varphi-\varphi_0
\end{bmatrix}
,&
\tilde{\boldsymbol u}
=
\begin{bmatrix}
	v_r \\
	\delta \\
\end{bmatrix}
$$

$$
\newcommand\c{\boldsymbol C}
\newcommand\d{\boldsymbol D}
\newcommand\t{\boldsymbol T}
\newcommand\i{\boldsymbol I}
\newcommand\x{\boldsymbol \xi}
\newcommand\p{\partial}
\newcommand\l{\bigg{|}_{\begin{array}{cc}\xi=\xi_0\\u=u_0\end{array}}}


\begin{aligned}

\c&=\frac{\p f}{\p\x}\l\t+\i

=\frac
{\p
    \begin{bmatrix}
        v_r\cos\varphi \\
        v_r\sin\varphi \\
        v_r\frac{\tan\delta}{L}
    \end{bmatrix}
}
{\p
    \begin{bmatrix}
        X \\
        Y \\
        \varphi
    \end{bmatrix}
}\l\t+\i
=
\begin{bmatrix}
    \frac{\p v_r\cos\varphi}{\p X} & \frac{\p v_r\cos\varphi}{\p Y} & \frac{\p v_r\cos\varphi}{\p\varphi} \\
    \frac{\p v_r\sin\varphi}{\p X} & \frac{\p v_r\sin\varphi}{\p Y} & \frac{\p v_r\sin\varphi}{\p\varphi} \\
    \frac{\p v_r\frac{\tan\delta}{L}}{\p X} & \frac{\p v_r\frac{\tan\delta}{L}}{\p Y} & \frac{\p v_r\frac{\tan\delta}{L}}{\p\varphi} \\
\end{bmatrix}\l\t+\i
=
\begin{bmatrix}
	0 & 0 & -v_r\sin\varphi \\
	0 & 0 &  v_r\cos\varphi \\
	0 & 0 & 0
\end{bmatrix}\t+
\begin{bmatrix}
	1 & 0 & 0 \\
	0 & 1 & 0 \\
	0 & 0 & 1
\end{bmatrix}

\\
&=
\begin{bmatrix}
	0 & 0 & -Tv_r\sin\varphi \\
	0 & 0 &  Tv_r\cos\varphi \\
	0 & 0 & 1
\end{bmatrix}
\end{aligned}
$$

$$
\newcommand\c{\boldsymbol C}
\newcommand\d{\boldsymbol D}
\newcommand\t{\boldsymbol T}
\newcommand\i{\boldsymbol I}
\newcommand\u{\boldsymbol u}
\newcommand\x{\boldsymbol \xi}
\newcommand\p{\partial}
\newcommand\l{\bigg{|}_{\begin{array}{cc}\xi=\xi_0\\u=u_0\end{array}}}

\d=\frac{
\p\begin{bmatrix}
	v_r\cos\varphi \\
	v_r\sin\varphi \\
	v_r\frac{\tan\delta}{L}
\end{bmatrix}}
{\p\begin{bmatrix}
	v_r \\
	\delta
\end{bmatrix}}T
=
\begin{bmatrix}
	\frac{\p v_r\cos\varphi}{\p v_r} & \frac{\p v_r\cos\varphi}{\p\delta} \\
	\frac{\p v_r\sin\varphi}{\p v_r} & \frac{\p v_r\sin\varphi}{\p\delta} \\
	\frac{\p v_r\frac{\tan\delta}{L}}{\p v_r} & \frac{\p v_r\frac{\tan\delta}{L}}{\p\delta} \\
\end{bmatrix}T
=
\begin{bmatrix}
	\cos\varphi & 0 \\
	\sin\varphi & 0 \\
	\frac{\tan\delta}{L} & \frac{v_r}{L\cos^2\delta}
\end{bmatrix}T
=
\begin{bmatrix}
	T\cos\varphi & 0 \\
	T\sin\varphi & 0 \\
	T\frac{\tan\delta}{L} & T\frac{v_r}{L\cos^2\delta}
\end{bmatrix}
$$



### 预测

$$
\begin{aligned}

\boldsymbol{\tilde\xi}_{(k+1)} &= {\boldsymbol{A}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{B\tilde{u}}_k\\

\boldsymbol{\tilde\xi}_{(k+2)} &= {\boldsymbol{A}\boldsymbol{\tilde{\xi}}}_{(k+1)} + \boldsymbol{B\tilde{u}}_{(k+1)} \\
&=\boldsymbol A({\boldsymbol{A}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{B\tilde{u}}_k) + \boldsymbol{B\tilde{u}}_{(k+1)} \\
&={\boldsymbol{A^2}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{AB\tilde{u}}_k + \boldsymbol{B\tilde{u}}_{(k+1)} \\

\boldsymbol{\tilde\xi}_{(k+3)} &= {\boldsymbol{A}\boldsymbol{\tilde{\xi}}}_{(k+2)} + \boldsymbol{B\tilde{u}}_{(k+2)} \\
&={\boldsymbol{A^3}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{A^2B\tilde{u}}_k + \boldsymbol{AB\tilde{u}}_{(k+1)} + \boldsymbol{B\tilde{u}}_{(k+2)} \\

\boldsymbol{\tilde\xi}_{(k+4)} &= {\boldsymbol{A}\boldsymbol{\tilde{\xi}}}_{(k+3)} + \boldsymbol{B\tilde{u}}_{(k+3)} \\
&={\boldsymbol{A^4}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{A^3B\tilde{u}}_k + \boldsymbol{A^2B\tilde{u}}_{(k+1)} + \boldsymbol{AB\tilde{u}}_{(k+2)} + \boldsymbol{B\tilde{u}}_{(k+3)} \\

\vdots \\

\boldsymbol{\tilde\xi}_{(k+Np)} &= {\boldsymbol{A}^{Np}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{A^3B\tilde{u}}_k + \boldsymbol{A^2B\tilde{u}}_{(k+1)} + \boldsymbol{AB\tilde{u}}_{(k+2)} + \boldsymbol{B\tilde{u}}_{(k+3)} \\

\end{aligned}

%\color[rgb]{0.5,0.6,0.7}
$$

写成矩阵形式：
$$
\left[
\begin{array}{r|r:rrrr}


\boldsymbol{\tilde{\xi}}_{(k+1)} & 
\boldsymbol{A}^1\boldsymbol{\tilde{\xi}}_{k} & 
\boldsymbol{A}^0\boldsymbol{B}\boldsymbol{\tilde u}_{k} \\



\boldsymbol{\tilde{\xi}}_{(k+2)} & 
\boldsymbol{A}^2\boldsymbol{\tilde{\xi}}_{k} & \boldsymbol{A}^1\boldsymbol{B}\boldsymbol{\tilde{u}}_{k} & 
\boldsymbol{A}^0\boldsymbol{B}\boldsymbol{\tilde u}_{(k+1)} \\

\boldsymbol{\tilde{\xi}}_{(k+3)} & 
\boldsymbol{A}^3\boldsymbol{\tilde{\xi}}_{k} & \boldsymbol{A}^2\boldsymbol{B}\boldsymbol{\tilde{u}}_{k} & 
\boldsymbol{A}^1\boldsymbol{B}\boldsymbol{\tilde{u}}_{(k+1)} &
\boldsymbol{A}^0\boldsymbol{B}\boldsymbol{\tilde u}_{(k+2)} \\

\boldsymbol{\tilde{\xi}}_{(k+4)} & \boldsymbol{A}^4\boldsymbol{\tilde{\xi}}_{k} & \boldsymbol{A}^{3}\boldsymbol{B}\boldsymbol{\tilde{u}}_{k} & 
\boldsymbol{A}^2\boldsymbol{B}\boldsymbol{\tilde{u}}_{(k+1)} & 
\boldsymbol{A}^1\boldsymbol{B}\boldsymbol{\tilde{u}}_{(k+2)} & 
\boldsymbol{A}^0\boldsymbol{B}\boldsymbol{\tilde u}_{(k+3)} \\


\end{array}
\right]
$$
总结规律：

1. $\boldsymbol{\tilde\xi}_k$只出现在第一项，它的系数$\boldsymbol A$的次幂等于预测的步数，该步数成为**预测时域**，用$Np$表示。
2. 从第二项开始没有$\boldsymbol{\tilde\xi}_k$，但都有$\boldsymbol B$，且$\boldsymbol B$的次幂均为1。第二项开始的任意一项可表示为$\boldsymbol{A}^{(n-Nc)}\boldsymbol{Bu}_{(k+Nc)}, n=Np-1$，$Nc$称为**控制时域**，且最大为值为$n$。
3. 从上面矩阵中可总结出新的模型：

$$
\boldsymbol{Y}=\boldsymbol{\phi\xi}_{k}+\boldsymbol{\theta U}
$$

$$
\boldsymbol{Y}=
\begin{bmatrix}
	\boldsymbol{\tilde\xi}_{(k+1)} \\
	\boldsymbol{\tilde\xi}_{(k+2)} \\
	\vdots \\
	\boldsymbol{\tilde\xi}_{(k+Np)} \\
\end{bmatrix}
,&
\boldsymbol{\phi}=
\begin{bmatrix}
	\boldsymbol{A}^1 \\
	\boldsymbol{A}^2 \\
	\vdots \\
	\boldsymbol{A}^{Np} \\
\end{bmatrix}
,&
\boldsymbol{\theta}=
\begin{bmatrix}
	\boldsymbol{A}^0 & \cdots \\
	\boldsymbol{A}^1 & \boldsymbol{A}^0 & \cdots \\
	\vdots \\
	\boldsymbol{A}^{Np-1} & \cdots & \boldsymbol{A}^{Np-Nc-1} & \boldsymbol{A}^0\\
\end{bmatrix} \boldsymbol{B}
,&
\boldsymbol{U}=
\begin{bmatrix}
	\boldsymbol{u}_{k} \\
	\boldsymbol{u}_{(k+1)} \\
	\vdots \\
	\boldsymbol{u}_{(k+Nc)} \\
\end{bmatrix}
$$

可知$\boldsymbol{\theta}$为下三角矩阵，且$\boldsymbol A$的次幂按列递增到$Np$、按行递减到$Np-Nc-1$。

### 优化

使模型以最小的控制变化量最快达到参考状态量。

目标函数：
$$
\newcommand\y{\boldsymbol Y}
\newcommand\u{\boldsymbol U}
\newcommand\r{\boldsymbol R}
\newcommand\q{\boldsymbol Q}

\min[(\y-\y_{ref})^T\q(\y-\y_{ref}) + \u^T\r\u] \\
$$
 其中，$\boldsymbol Q$和$\boldsymbol R$分别为状态量和控制量的权重。

令$\boldsymbol{E=\phi\xi}_{k}-\boldsymbol{\phi\xi}_{ref}=\boldsymbol{\phi\xi}_{k}-\boldsymbol{Y}_{ref}$，则得$\boldsymbol{Y-Y}_{ref}=\boldsymbol{E+\theta U}$，于是目标函数变为：
$$
\newcommand\E{\boldsymbol E}
\newcommand\U{\boldsymbol U}
\newcommand\R{\boldsymbol R}
\newcommand\Q{\boldsymbol Q}
\newcommand\th{\boldsymbol\theta}

\begin{aligned}

&\min[(\boldsymbol{E+\theta U})^T \boldsymbol{Q} (\boldsymbol{E+\theta U}) + 
\boldsymbol{U}^T\boldsymbol{RU}] \\
\\
=&\min[(\E^T + (\th\U)^T)(\Q\E+\Q\th\U) + \U^T\R\U] \\
=&\min[{\color{silver}\bcancel{\E^T\Q\E}} + {\color[rgb]{0.2, 0.5, 0.8}\E^T\Q(\th\U) + (\th\U)^T\Q\E} + (\th\U)^T\Q(\th\U) + \U^T\R\U] \\
=&\min[{\color[rgb]{0.2, 0.5, 0.8}2\E^T\Q(\th\U)} + (\th\U)^T\Q(\th\U) + \U^T\R\U]\\
=&\min[{\color[rgb]{0.2, 0.5, 0.8}2\E^T\Q(\th\U)} + \U^T\th^T\Q\th\U + \U^T\R\U]\\
=&\min[{\color[rgb]{0.2, 0.5, 0.8}2\E^T\Q(\th\U)} + \U^T(\th^T\Q\th + \R)\U]\\


\end{aligned}
$$
不计与$\boldsymbol U$没有关系的常数项$\boldsymbol E^T \boldsymbol{QE}$。推导过程用到矩阵转置运算法则：$\boldsymbol{(A+B)}^T=\boldsymbol{A}^T+\boldsymbol{B}^T$，$\boldsymbol{A}^T\boldsymbol{B}=\boldsymbol{B}^T\boldsymbol{A}$，$\boldsymbol{(AB)}^T=\boldsymbol{B}^T\boldsymbol{A}^T$。

即得到标准形式：
$$
\newcommand\j{\boldsymbol J}
\newcommand\x{\boldsymbol X}
\newcommand\h{\boldsymbol H}

\min\j=\frac{1}{2}\x^T\h\x + f^T\x
$$


### 反馈控制

[底盘控制交流群](https://www.bilibili.com/video/BV1Pv4y127Vc/?spm_id_from=333.788&vd_source=31c586dfbb435719033978621d59898c)

[Ally](https://www.bilibili.com/video/BV1pV411n7uN/?spm_id_from=333.999.0.0&vd_source=31c586dfbb435719033978621d59898c)修改龚的代码，有不等式约束
