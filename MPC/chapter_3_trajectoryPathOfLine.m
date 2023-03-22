% =================
% 直线轨迹跟踪过程
% =================


clc;
clear all;


% 参考轨迹生成
N=100;                                              % 参考轨迹点数量
T=0.05;                                             % 采样周期
Xout=zeros(N,3);                                    % zeros(d1,d2)是产生一个的d1*d2的全0矩阵
Tout=zeros(N,1);
for k=1:1:N                                         % 以增量为1递增到N
    Xout(k,1)=k*T;                                  % 将每个周期点整数倍的横坐标赋给k
    Xout(k,2)=2;
    Xout(k,3)=0;
    Tout(k,1)=(k-1)*T;
end


Nx=3;                                               % 状态量个数
Nu =2;                                              % 控制量个数
Tsim =20;                                           % 仿真时间 20s
X0 = [0 0 pi/3];                                    % 车辆初始状态[x, y, angle]
[Nr,Nc] = size(Xout);                               % Nr输出的行数 Nc输出的列数


% 车辆参数
c = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
L = 1;                                              % 车辆轴距
Rr = 1;
w = 1;



vd1 = Rr*w;                                         % 参考系统的纵向速度
vd2 = 0;                                            % 参考系统的前轮偏角
x_real=zeros(Nr,Nc);
x_piao=zeros(Nr,Nc);
u_real=zeros(Nr,2);
u_piao=zeros(Nr,2);
x_real(1,:)=X0;                                     % 指x矩阵的第一行所有列，:表示所有的
x_piao(1,:)=x_real(1,:)-Xout(1,:);
X_PIAO=zeros(Nr,Nx*Tsim);
XXX=zeros(Nr,Nx*Tsim);                              % 用于保存每个时刻预测的所有状态值
q=[1 0 0;0 1 0;0 0 0.5];
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