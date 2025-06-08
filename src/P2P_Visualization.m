%% 自由运动
run('P2P_free_motions.m');

%% 可视化结果
% figure('Position', [100, 100, 1400, 800]);
% joint_names = {'肩水平内收', '肩屈伸', '肩部内外旋转', '肘关节屈伸', '前臂旋前旋后', '腕屈伸', '腕桡尺偏斜'};
% for dim = 1:n_dims
%     subplot(2,4,dim);
%     plot(t_prime, q_traj(:, dim)), hold on;
%     scatter([0, t_prime_m], [q0(dim), qm(dim)], 100, 'filled');
%     title(joint_names{dim});
%     xlabel('时间 (s)'); ylabel('角度 (rad)');
%     grid on; legend('轨迹', '目标点');
% end
% sgtitle('基于函数主成分的点对点自由运动生成');

% 定义DH参数（使用Robotics Toolbox）
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0);    % 肩水平内收
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2);     % 肩屈伸
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0);        % 肩部内外旋转
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2);     % 肘关节屈伸
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0);        % 前臂旋前旋后
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0);     % 腕屈伸
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0);        % 腕桡尺偏斜

% 创建机械臂模型
robot = SerialLink(L, 'name', '7-DOF Humanoid Arm');

% 假设已有时间序列和对应的关节角度数据
% 时间点（单位：秒）
sampling_frequency = 10; % 采样频率为 10 Hz
total_time = 12; % 总时长为 12 秒
num_joints = 7; % 7 个关节
time_points = linspace(0, total_time, 120); % 生成 120 个时间点
numericArray = transpose(numericArray);

% 定义高采样率的连续时间序列（用于插值）
t_fine = linspace(0, total_time, 1000); % 生成 1000 个时间点

q_traj = q_traj';

% 对每个关节进行插值（使用样条插值）
q_traj1 = zeros(length(t_fine), num_joints); % 初始化插值后的轨迹矩阵
for i = 1:num_joints
    q_traj1(:, i) = interp1(time_points, q_traj(i, :), t_fine, 'spline');
end

q_traj2 = zeros(length(t_fine), num_joints); % 初始化插值后的轨迹矩阵
for i = 1:num_joints
    q_traj2(:, i) = interp1(time_points, numericArray(i, :), t_fine, 'spline');
end

robot = SerialLink(L, 'name', '7-DOF Humanoid Arm');

% --------------------------
% 使用Robotics Toolbox进行动画演示
% --------------------------

figure('Position', [440, 300, 560, 420]); 
% 绘制第一个轨迹
robot.plot(q_traj1, 'trail', 'r-', 'view', [-70 10], 'fps', 500);
figure('Position', [1000, 300, 560, 420]); 
% 绘制第二个轨迹，保留第一个轨迹
robot.plot(q_traj2, 'trail', 'b--', 'view', [-70 10], 'fps', 500);