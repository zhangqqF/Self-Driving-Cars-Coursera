% =================
% 直线轨迹跟踪过程
% =================


clc;
clear all;


% 
% 参考轨迹生成
% 
N=100;                                                % 参考轨迹点数量
T=0.05;                                               % 采样周期
Xout=zeros(N,3);                                      % zeros(d1,d2)是产生一个的d1*d2的全0矩阵
Tout=zeros(N,1);

                % 构造参考点和采样时间矩阵
for k=1:1:N
    Xout(k,1)=k*T;
    Xout(k,2)=2;
    Xout(k,3)=0;
    Tout(k,1)=(k-1)*T;
end
% Xout(k,3)=0
% Tout(k,1)=(k-1)*T


Nx=3;                                               % 状态量个数，[x, y, phi]
Nu =2;                                              % 控制量个数，[v, delta]
Tsim =20;                                           % 仿真时间 20s
X0 = [0 0 pi/3];                                    % 车辆初始状态
[Nr,Nc] = size(Xout);                               % 获取参考轨迹点矩阵的行、列数


% 车辆参数
c = eye(4);                                         % 构造单位矩阵
L = 1;                                              % 车辆轴距
% Rr = 1;
% w = 1;
% vd1 = Rr*w;                                         % 参考系统的纵向速度
vd1 = 1;                                            % 参考系统的纵向速度
vd2 = 0;                                            % 参考系统的前轮偏角


% 矩阵定义
x_real=zeros(Nr,Nc);                                % 车辆实际运行轨迹的矩阵 [x, y, phi]
x_piao=zeros(Nr,Nc);                                % 车辆实际运行轨迹与参考轨迹的误差矩阵 [x-x_ref, y-y_ref,phi-phi_ref]
u_real=zeros(Nr,2);                                 % 车辆控制量矩阵 [v, delta]
u_piao=zeros(Nr,2);                                 % 车辆控制量误差矩阵 [v-0, delta-0]

x_real(1,:)=X0;                                     % 赋予初始状态
x_piao(1,:)=x_real(1,:)-Xout(1,:);                  % 计算误差
X_PIAO=zeros(Nr,Nx*Tsim);
XXX=zeros(Nr,Nx*Tsim);                              % 用于保存每个时刻预测的所有状态值

% 构造权重矩阵
% 权重矩阵是对角矩阵
q=[1 0 0; 0 1 0; 0 0 0.5];                          % x, y, phi的权重分别为1, 1, 0.5
Q_cell=cell(Tsim,Tsim);
for i=1:1:Tsim
    for j=1:1:Tsim
        if i==j
            Q_cell{i,j}=q;
        else 
            Q_cell{i,j}=zeros(Nx,Nx);
        end 
    end
end
Q=cell2mat(Q_cell);                                 % 状态量权重矩阵
R=0.1*eye(Nu*Tsim,Nu*Tsim);                         % 控制量权重矩阵



% 控制主体
for i=1:1:Nr
    t_d =Xout(i,3);                                 % 获取每一步的航向角phi
    a=[1    0   -vd1*sin(t_d)*T;                    % 离散化后的A矩阵  ksai_piao(k+1) = Aksai_piao + Bu_piao
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;                              % B矩阵
       sin(t_d)*T   0;
       0            T;];     
    A_cell=cell(Tsim,1);
    B_cell=cell(Tsim,Tsim);
    for j=1:1:Tsim
        A_cell{j,1}=a^j;
        for k=1:1:Tsim
           if k<=j
                B_cell{j,k}=(a^(j-k))*b;
           else
                B_cell{j,k}=zeros(Nx,Nu);
           end
        end
    end
    A=cell2mat(A_cell)
    B=cell2mat(B_cell)
    
    return
    
    
end