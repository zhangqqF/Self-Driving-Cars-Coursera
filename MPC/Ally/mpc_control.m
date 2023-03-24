

function [delta_real, v_real, idx, error_y, U] = mpc_control(x, y, yaw, refPos_x, refPos_y, refPos_yaw, refPos_k, dt, L, U, target_v)


% MPC预设参数
Nx = 3;                         % 状态量个数
Nu = 2;                         % 控制量个数
Np = 60;                        % 预测步长
Nc = 30;                        % 控制步长
row = 10                        % 松弛因子
Q = 100*eye(Np, Nx);
R = 1*eye(Np, Nu);


% 控制量约束
umin = [-0.2; -0.54];
umax = [0.2; 0.332];
delta_umin = [-0.05; -0.64];
delta_umax = [0.05; 0.64];


% % 原运动学误差状态空间方程的相关矩阵
% 计算参考控制量
[idx, error_y] = calc_target_index(x, y, refPos_x, refPos_y);
curvature = refPos_k(idx);
delta_r = atan(L*curvature);                                            % 阿克曼转向定理
v_r = target_v;

% 实际状态量和参考状态量
X_real = [x, y, yaw];
Xr = [refPos_x(idx), refPos_y(idx), refPos_yaw(idx)];


% A、B两个矩阵
a = [1 0 -v_r*sin(yaw)*dt;
     0 1 v_r*cos(yaw)*dt;
     0 0 1];
b = [cos(yaw)*dt 0 0;
     sin(yaw)*dt 0 0;
     tan(yaw)*dt/L v_r*dt/(L*(cos(delta_r)^2))];


% % 新运动学误差状态空间方程的相关矩阵
% 新的状态量
kesi = zeros(Nx+Nu, 1);
kesi(1:Nx) = X_real - Xr;
kesi(Nx+1, end) = U;

% 新的A矩阵
A_cell = cell(2);
A_cell{1, 1} = a;
A_cell{1, 2} = b;
A_cell{2, 1} = zeros(Nu, Nx);
A_cell{2, 2} = eye(Nu);
A = cell2mat(A_cell);

% 新的B矩阵
B_cell = cell(2, 1);
B_cell{1, 1} = b;
B_cell{2, 1} = eye(Nu);
B = cell2mat(A_cell);

% 新的C矩阵
C = [eye(Nx), zeros(Nx, Nu)];


% PHI矩阵
PHI_cell = cell(Np, 1);
for i=1:Np
    PHI_cell{i, j} = C*A^i;
end
PHI = cell2mat(PHI_cell);

% THETA矩阵
THETA_cell = cell(Np, Nc);
for i=1:Np
    for j=1:Nc
        if i >= j
            THETA_cell{i, j} = C*A^(i-j)*B;
        else
            THETA_cell{i, j} = zeros(Nx, Nu);
        end
    end
end
THETA = cell2mat(THETA_cell);


% % 二次型目标函数的相关矩阵
% H矩阵
H_cell = cell(2);
H_cell{1, 1} = THETA'*Q*THETA + R;
H_cell{1, 2} = zeros(Nu*Nc, 1);
H_cell{2, 1} = zeros(1, Nu*Nc);
H_cell{2, 2} = row;
H = cell2mat(H_cell);

% E矩阵
E = PHI*kesi;

% g矩阵
g_cell = cell(1);
g_cell{1, 1} = E'*Q*THETA;
g_cell{1, 2} = 0;                                                         % 行数为使和H列数匹配，添加一列0
g = cell2mat(g_cell);


%% 约束条件的相关矩阵
% A_I矩阵
A_t = zeros(Nc);                                                    % 下三角方阵
for i=1:Nc
    A_t(1, 1:i) = 1;
end
A_I = kron(A_t, eye(Nu))

% Ut矩阵
Ut = kron(ones(Nc, 1), U);

% 控制量和控制变化量的约束
Umin = kron(ones(Nc, 1), umin);
Umax = kron(ones(Nc, 1), umax);
delta_Umin = kron(ones(Nc, 1), delta_umin);
delta_Umax = kron(ones(Nc, 1), delta_umax);


% 用quadprog函数不等式约束Ax <= b的矩阵A
A_cons_cell = {A_I, zeros(Nu*Nc, 1); -A_I, zeros(Nu*Nc, 1)};         % 增加一列0以匹配H的列数
A_cons = cell2mat(A_cons_cell);

% 用quadprog函数不等式约束Ax <= b的向量b
b_cons_cell = {Umax-Ut; -(Umax-Ut)};
b_cons = cell2mat(b_cons_cell);

% △U的上下边界约束
lb = [delta_Umin; 0];
ub = [delta_Umax; 1];


%% 开始求解
opt = optimoptions('quadprog', 'Display', 'iter', 'MaxIteration', 100, 'TolFun', 1e-16);
delta_U = quadprog(H, g, A_cons, b_cons, [], [], lb, ub, [], opt);


%% 计算输出
delta_v_tilde = delta_U(1);
delta_delta_tilde = delta_U(2);

% 更新这一时刻的状态量
U(1) = kesi(4) + delta_v_tilde;
U(2) = kesi(5) + delta_delta_tilde;

% 求解真正的控制量
v_real = U(1) + v_r;
delta_real = U(2) + delta_r;


end