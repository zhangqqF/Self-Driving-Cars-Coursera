% =================
% 直线轨迹跟踪过程
% =================


clc;
clear all;
close all;


% 参考轨迹生成
N = 100;                                            % 参考轨迹点数量
T = 0.05;                                           % 采样周期
ksai = zeros(N,3);                                  % zeros(d1,d2)是产生一个的d1*d2的全0矩阵
Tout = zeros(N,1);
for k = 1:1:N                                       % 以增量为1递增到N
    ksai(k,1) = k*T;                                % x坐标为0.05, 0.10, 0.115, ..., 5
    ksai(k,2) = 2;                                  % y坐标恒为2
    ksai(k,3) = 0;                                  % yaw为0
    Tout(k,1) = (k-1)*T;
end

% Tracking a constant reference trajectory
Nx = 3;                                             % 状态量个数
Nu = 2;                                             % 控制量个数
Tsim = 20;                                          % 仿真时间 20s

[Nr, Nc] = size(ksai);                              % Nr输出的行数 Nc输出的列数


% 车辆参数
c = [1 0 0 0; 0 1 0 0;0 0 1 0; 0 0 0 1];
L = 1;                                              % 车辆轴距

v = 1;                                              % 参考系统的纵向速度
yaw = 0;                                            % 参考系统的前轮偏角
ksai_real = zeros(Nr, Nc);
ksai_piao = zeros(Nr, Nc);
u_real = zeros(Nr, 2);
u_piao = zeros(Nr, 2);
ksai_real(1, :) = [0 0 pi/3];                       % 车辆初始状态
ksai_piao(1, :) = ksai_real(1, :) - ksai(1, :);
X_PIAO = zeros(Nr, Nx*Tsim);
XXX = zeros(Nr, Nx*Tsim);                           % 用于保存每个时刻预测的所有状态值


% 权重矩阵
q = [1 0 0; 0 1 0; 0 0 0.5];
Q_cell = cell(Tsim, Tsim);
for i = 1:1:Tsim
    for j = 1:1:Tsim
        if i == j
            Q_cell{i, j} = q;
        else 
            Q_cell{i, j} = zeros(Nx, Nx);
        end 
    end
end
Q = cell2mat(Q_cell);                               % 状态量权重矩阵
R = 0.1*eye(Nu*Tsim, Nu*Tsim);                      % 控制量权重矩阵


for i = 1:1:Nr
    delta = ksai(i,3);
    A = [1    0   -v*sin(delta)*T;
         0    1   v*cos(delta)*T;
         0    0   1;];
    B = [cos(delta)*T       0;
         sin(delta)*T       0;
         tan(delta)*T/L     v*T/(L*cos(delta)^2);];     
    PHI_cell = cell(Tsim, 1);
    THETA_cell = cell(Tsim, Tsim);
    for j = 1:1:Tsim
        PHI_cell{j, 1} = A^j;
        for k = 1:1:Tsim
           if k <= j
                THETA_cell{j, k} = (A^(j-k))*B;
           else
                THETA_cell{j, k} = zeros(Nx, Nu);
           end
        end
    end
    PHI = cell2mat(PHI_cell);
    THETA = cell2mat(THETA_cell);
    
    H = 2*(THETA'*Q*THETA+R);
    f = 2*THETA'*Q*PHI*ksai_piao(i,:)';              % f^T矩阵
    A_cons = [];
    b_cons = [];
    lb = [-1; -1];                                   % 控制量下界
    ub = [1; 1];                                     % 控制量上界
    tic                                              % 开始计时
    [X, fval(i,1), exitflag(i,1), output(i,1)] = quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
    % quadprog
    % A*X <= b          不等式约束
    % Aeq*X = beq       没有不等式约束，则Aeq和beq均为[]
    % lb <= X <= up     上下界约束
    toc                                              % 结束计时
    X_PIAO(i, :) = (PHI*ksai_piao(i, :)' + THETA*X)';
    if i+j < Nr
         for j = 1:1:Tsim
             XXX(i, 1+3*(j-1)) = X_PIAO(i, 1+3*(j-1)) + ksai(i+j, 1);
             XXX(i, 2+3*(j-1)) = X_PIAO(i, 2+3*(j-1)) + ksai(i+j, 2);
             XXX(i, 3+3*(j-1)) = X_PIAO(i, 3+3*(j-1)) + ksai(i+j, 3);
         end
    else
         for j = 1:1:Tsim 
             XXX(i, 1+3*(j-1)) = X_PIAO(i, 1+3*(j-1))+ksai(Nr, 1);
             XXX(i, 2+3*(j-1)) = X_PIAO(i, 2+3*(j-1))+ksai(Nr, 2);
             XXX(i, 3+3*(j-1)) = X_PIAO(i, 3+3*(j-1))+ksai(Nr, 3);
         end
    end
    u_piao(i, 1) = X(1, 1);
    u_piao(i, 2) = X(2, 1);
    Tvec = [0:0.05:4];
    X00 = ksai_real(i,:);
    vd11 = v + u_piao(i,1);
    vd22 = yaw + u_piao(i,2);
    XOUT = dsolve('Dx-vd11*cos(z)=0','Dy-vd11*sin(z)=0','Dz-vd22=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    t = T; 
    ksai_real(i+1, 1) = eval(XOUT.x);
    ksai_real(i+1, 2) = eval(XOUT.y);
    ksai_real(i+1, 3) = eval(XOUT.z);
    if(i < Nr)
        ksai_piao(i+1,:) = ksai_real(i+1,:) - ksai(i+1,:);
    end
    u_real(i, 1) = v + u_piao(i, 1);
    u_real(i, 2) = yaw + u_piao(i, 2);
    
    figure(1);
    plot(ksai(1:Nr, 1), ksai(1:Nr, 2));
    hold on;
    plot(ksai_real(i,1), ksai_real(i,2),'bo');
    title('跟踪结果对比');
    xlabel('横向位置X');
    axis([-1 5 -1 3]);
    ylabel('纵向位置Y');
    hold on;
    for k = 1:1:Tsim
        X(i, k+1) = XXX(i, 1+3*(k-1));
        Y(i, k+1) = XXX(i, 2+3*(k-1));
    end
    X(i, 1) = ksai_real(i, 1);
    Y(i, 1) = ksai_real(i, 2);
    plot(X(i,:), Y(i,:),'y')
    hold on;
    
end


