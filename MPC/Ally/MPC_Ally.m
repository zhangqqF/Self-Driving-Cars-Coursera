% 利用MPC跟踪轨迹
% 原作者：Ally
% Data：2021/04/03

% zhangqq Mar-4, 2023

clc; clear; close all;


load path.mat


% 初始参数
Kp = 1;
dt = 0.1;                               % 时间步长，即采样时间
L = 2.9;                                % 轴距
max_steer = 60*pi/180;                  % in rad
target_v = 30/3.6


% 参考轨迹
refPos_x = path(:, 1);
refPos_y = path(:, 2);
refPos   = [refPos_x, refPos_y];


% 计算一阶导
for i = 1:length(refPos_x)-1
    refPos_d(i) = (refPos(i+1, 2) - refPos(i, 2)) / (refPos(1, i+1) - refPos(1, i));
end
refPos_d(end+1) = refPos_d(end);

% 计算二阶导
for i = 2:length(refPos_x)-1
    refPos_dd(i) = (refPos(i+1, 2) - 2*refPos(i, 2) + refPos(i-1, 2)) / (0.5*(-refPos(i-1, 1)) + refPos(1, i));
end
refPos_dd(1) = refPos_dd(2);
refPos_dd(length(refPos_x)) = refPos_dd(length(refPos_x)-1);


% 计算曲率
for i = 1:length(refPos_x)-1
    k(i) = (refPos_dd(i)) / (1 + refPos_d(i)^2)^1.5;
end

refPos_x = refPos_x';
refPos_y = refPos_y';
refPos_yaw = atan(refPos_d');
refPos_k = k';


% 绘图
figure
plot(refPos_x, refPos_y, 'r-')
hold on;


% 主程序
x = 0.1;
y = -0.1;
yaw = 0.1;
v = 0.1;
U = [0.01; 0.01];                                       % 初始控制量
ind = 0;                                                % 索引
pos_actual = [x, y];                                    % 存放每一步的实际位置

while ind < length(refPos_x)
    % 调用MPC控制器
    [delta, v, ind, U] = mpc_control(x, y, yaw, refPos_x, refPos_y, refPos_yaw, refPos_k, dt, L, U, target_v)

    % 误差太大，退出程序
    if abs(e) > 3
        fprintf('误差过大，退出程序！\n');
        break
    end
end


% 速度P控制器
a = Kp*(target_v-v);
