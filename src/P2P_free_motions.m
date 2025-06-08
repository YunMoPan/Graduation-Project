%% fPCAs计算
run('fPCAs.m');

%% 点对点自由运动生成
% 拿第一组数据举例
numericArray = joint_angles; % 转为弧度
q0 = numericArray(1,:);                 % （1×7）
qm = numericArray(end,:);

t_prime_m = t_fin;  % 使用总时间作为运动持续时间

% 提取第一个主成分（M=1）
S1 = S{1};  % M×7的主成分基函数

% 计算S0和S1在初始和最终时刻的值
S0_0 = S0(1, :);      % 初始时刻S0值（1×7）
S0_tm = S0(end, :);    % 最终时刻S0值（1×7）
S1_0 = S1(1, :);       % 初始时刻S1值（1×7）
S1_tm = S1(end, :);    % 最终时刻S1值（1×7）

% 计算α1（公式15）
delta_q = qm - q0;                          % 姿态差
delta_S0 = S0_tm - S0_0;                    % S0变化量
delta_S1 = S1_tm - S1_0;                    % S1变化量
alpha1 = (delta_q - delta_S0) ./ delta_S1;  % 阿达玛逆即元素级除法

% 计算全局平均姿态tilde_q（公式16-17）
tilde_q = q0 - alpha1 .* S1_0 - S0_0;

%% 生成运动轨迹
num_points = 120;  % 轨迹点数
t_prime = linspace(0, t_prime_m, num_points)';  % 时间向量

% % 插值基函数
% S0_interp = interp1(t, S0, t_prime, 'spline');  % 样条插值
% S1_interp = interp1(t, S1, t_prime, 'spline');

% 构建完整轨迹（公式10）
q_traj = zeros(num_points, n_dims);
for i = 1:num_points
    q_traj(i, :) = tilde_q + ...              % 全局平均姿态
                  S0(i, :) + ...       % 零阶协同效应
                  alpha1 .* S1(i, :); % 主成分贡献
end

% %% 验证边界条件
% % 初始时刻误差
% error_start = norm(q_traj(1, :) - q0);
% 
% % 最终时刻误差
% error_end = norm(q_traj(end, :) - qm);
% 
% fprintf('初始姿态误差：%.2e\n最终姿态误差：%.2e\n', error_start, error_end);