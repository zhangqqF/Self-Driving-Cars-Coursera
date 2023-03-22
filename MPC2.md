

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
\dot{\xi}=f(\xi{_0}, u_0)+
\frac{\partial f}{\partial \xi} \bigg|_{\begin{array}{cc}\xi=\xi_{0} \\ u=u_0 \end{array}} (\xi-\xi{_0})+
\frac{\partial f}{\partial u} \bigg|_{\begin{array}{cc}\xi=\xi_{0} \\ u=u_0 \end{array}} (u-u_0) \tag{3}

% github not support \begin{split}
$$
令$A=\frac{\partial{f}}{\partial\xi}\bigg{|}_{\begin{matrix}\xi=\xi_0 \\ u=u_0\end{matrix}}$，$B=\frac{\partial{f}}{\partial{u}}\bigg{|}_{\begin{matrix}\xi=\xi_0 \\ u=u_0\end{matrix}}$，再将式（2）减式（3），得：
$$
(\dot\xi-\dot\xi_{0})+A(\xi-\xi_{0})+B(u-u_0)=0
$$
由上式可看出$(\dot\xi-\dot\xi_{0})$、$(\xi-\xi_{0})$、$(u-u_0)$分别为$\dot\xi$、$\xi$、$u$相对于$\dot\xi_0$、$\xi_0$、$u_0$的误差。实际上，$\dot\xi_0$、$\xi_0$、$u_0$就是参考点，MPC控制的目标就是向参考点对齐。于是就得到了误差的线性化模型：
$$
\boldsymbol{\dot{\tilde{\xi}}=A\tilde{\xi} + B\tilde{u}} \tag{4}
$$
$\tilde i$表示误差，其中$\tilde{\dot\xi}=\dot\xi-\dot\xi_0$、$\tilde\xi=\xi-\xi_0$、$\tilde u=u-u_0$。

### 离散化

采用欧拉前向离散化，令$\boldsymbol {\dot{\tilde\xi}} = \frac{\boldsymbol{\tilde\xi}_{(k+1)}-\boldsymbol{\tilde\xi}_{k}} {T}$，$T$为采样时间，于是：
$$
\boldsymbol{\tilde\xi}_{(k+1)} = {(T\boldsymbol{A}+\boldsymbol{I})\boldsymbol{\tilde{\xi}}}_k + T\boldsymbol{B\tilde{u}}
$$
令$\boldsymbol{C}=T\boldsymbol{A}+\boldsymbol{I}$，$\boldsymbol{D}=T\boldsymbol{B}$上式亦可写成：
$$
\boldsymbol{\tilde\xi}_{(k+1)} = {\boldsymbol{C}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{D\tilde{u}} \tag{5}
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
	\boldsymbol{A}^{Np-1} & \cdots & \boldsymbol{A}^{Np-Nc} & \boldsymbol{A}^0\\
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

可知$\boldsymbol{\theta}$为下三角矩阵。

### 优化

目的是使模型以最小的控制变化量最快达到参考状态量。

目标函数：
$$
\min[(\boldsymbol{Y-Y}_{ref})^T {\boldsymbol Q(\boldsymbol Y- \boldsymbol Y_{ref})} + 
\boldsymbol{U}^T\boldsymbol{RU}]
$$
 其中，$\boldsymbol Q$和$\boldsymbol R$分别为状态量和控制量的权重。

令$\boldsymbol{E=\phi\xi}_{k}-\boldsymbol{\phi\xi}_{ref}=\boldsymbol{\phi\xi}_{k}-\boldsymbol{Y}_{ref}$，则得$\boldsymbol{Y-Y}_{ref}=\boldsymbol{E+\theta U}$，于是目标函数变为：
$$
\begin{aligned}

&\min[(\boldsymbol{E+\theta U})^T \boldsymbol{Q} (\boldsymbol{E+\theta U}) + 
\boldsymbol{U}^T\boldsymbol{RU}] \\
\\
=
&\min[(E^T + 2E\theta U + (\theta U)^T)(QE+Q\theta U) + U^TRU]\\
=
&\min[]\\

\end{aligned}
$$
![image-20230322173535070](F:\carla\New folder (2)\img\00)

### 反馈控制

