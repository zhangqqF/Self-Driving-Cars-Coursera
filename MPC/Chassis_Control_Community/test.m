
% MATLAB中用元胞数组来解决矩阵嵌套的问题
clc;
clear all;


% 构造离散化后的A矩阵
% [0 0 -tvsin(phi)
%  0 0 -tvcos(phi)
%  0 0     1      ]
A_cell = cell(2, 2);                                % 创建元胞数组
A_cell{1, 1} = eye(3);                              % 元胞赋值
A_cell{1, 2} = ones(3, 2);
A_cell{2, 1} = zeros(2, 3);
A_cell{2, 2} = eye(2);
A = cell2mat(A_cell);                                % 元胞数组转为一般矩阵 5x5

% 构造B矩阵
B = zeros(5, 2);



% 构造预测后的PHI矩阵
% [A^1
%  A^2
%  ...
%  A^Np]
Np = 10;
Nc = 6;
Ns = 3;
Nu = 2;
C = ones(3, 5);
PHI_cell = cell(Np, 1);
for i = 1:1:Np
    PHI_cell{i, 1} = C*A^Np;
end
PHI = cell2mat(PHI_cell);


% 构造预测后的THETA矩阵
% [A^0
%  A^1  A^0
%  A^2  A^1  A^0
%  A^Np ...      A^(Np-Nc-1)]
THETA_cell = cell(Np, Nc);
for i=1:1:Np
    for j=1:1:Nc
        if i >= j
            THETA_cell{i, j} = C*A^(i-j)*B;               % 3x5  5x5  5x2 矩阵相乘ixj*txk=ixk,j=t
        else
            THETA_cell{i, j} = zeros(3, 2);               % 注意矩阵维度与i>=j时相同
        end
    end
end
THETA = cell2mat(THETA_cell);


% 二次规划
% 两种方法：元胞数组、克罗克内积

B(1,1)=1
kron(A, B)
size(kron(A, B))
A
B

% 二次规划的两种解法：有效集法（只能解不等式约束）、内点法

