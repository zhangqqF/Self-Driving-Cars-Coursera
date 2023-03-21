

# MPC - Model Predictive Control

> zhangqq  
> Mar-16, 2023  
> Chongqing

---



- [MPC - Model Predictive Control](#mpc---model-predictive-control)
  - [车辆运动方程](#车辆运动方程)
    - [MPC模型线性化](#mpc模型线性化)
    - [MPC模型离散化](#mpc模型离散化)
    - [车辆运动方程线性离散化](#车辆运动方程线性离散化)

	
	

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

### MPC模型线性化

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

### MPC模型离散化

采用欧拉前向离散化，令$\boldsymbol {\dot{\tilde\xi}} = \frac{\boldsymbol{\tilde\xi}_{(k+1)}-\boldsymbol{\tilde\xi}_{k}} {T}$，$T$为采样时间，于是：
$$
\boldsymbol{\tilde\xi}_{(k+1)} = {(T\boldsymbol{A}+\boldsymbol{I})\boldsymbol{\tilde{\xi}}}_k + T\boldsymbol{B\tilde{u}}
$$
令$\boldsymbol{C}=T\boldsymbol{A}+\boldsymbol{I}$，$\boldsymbol{D}=T\boldsymbol{B}$上式亦可写成：
$$
\boldsymbol{\tilde\xi}_{(k+1)} = {\boldsymbol{C}\boldsymbol{\tilde{\xi}}}_k + \boldsymbol{D\tilde{u}} \tag{5}
$$

### 车辆运动方程线性离散化

