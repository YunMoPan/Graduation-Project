clear; close all; clc;

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

% 显示机械臂参数
robot.display();
robot.teach;

drawnow; 
set(gcf, 'Position',[160 440 560 420]); % [左 下 宽 高]，单位：像素

% 加载数据
file_address = ['C:\Users\maircal\Desktop\毕设\UE_ADLdatabase-master\ADL017\ADL017CL1angles.csv'];
[num,txt,raw] = xlsread(file_address);
rows = size(num, 1);
data = readcell(file_address);
data = data(2:rows+1, 5:11);           % 提取有效数据区域
numericArray = cell2mat(data);       % 转换为数值数组
numericArray = deg2rad(numericArray); % 转为弧度

%% 定义时间轴（假设原始时间为均匀分布）
original_time = linspace(0, 1, size(numericArray, 1))'; % 原始时间轴 [0,1] 区间
target_time = linspace(0, 1, 120)';                 % 目标时间轴（120点）

%% 初始化结果矩阵
joint_angles = zeros(120, 7);

%% 逐列进行三次样条插值
for col = 1:7
    joint_angles(:, col) = interp1(...
        original_time, ...       % 原始时间轴
        numericArray(:, col), ...    % 当前列数据
        target_time, ...         % 目标时间轴
        'spline', ...            % 三次样条插值
        'extrap' ...             % 允许外推（可选）
    );
end

% 假设已有时间序列和对应的关节角度数据
% 时间点（单位：秒）
sampling_frequency = 10; % 采样频率为 10 Hz
total_time = 12; % 总时长为 12 秒
num_joints = 7; % 7 个关节
time_points = linspace(0, total_time, 120); % 生成 120 个时间点
joint_angles = transpose(joint_angles);

% 定义高采样率的连续时间序列（用于插值）
t_fine = linspace(0, total_time, 1000); % 生成 1000 个时间点

% 对每个关节进行插值（使用样条插值）
q_traj = zeros(length(t_fine), num_joints); % 初始化插值后的轨迹矩阵
for i = 1:num_joints
    q_traj(:, i) = interp1(time_points, joint_angles(i, :), t_fine, 'spline');
end

% --------------------------
% 可选：可视化轨迹
% --------------------------
figure('Position',[850 560 560 420]);
for i = 1:num_joints
    subplot(4, 2, i);
    plot(time_points, joint_angles(i, :), 'ro', 'MarkerSize', 8); % 原始数据点
    hold on;
    plot(t_fine, q_traj(:, i), 'b-', 'LineWidth', 1.5); % 插值后的轨迹
    title(['关节 ', num2str(i)]);
    xlabel('时间 (秒)');
    ylabel('角度 (弧度)');
    grid on;
end
sgtitle('关节角度轨迹插值');

% pause(10);

% --------------------------
% 使用Robotics Toolbox进行动画演示
% --------------------------
% 创建第三个窗口
figure('Position',[850 40 560 420], 'Color', 'white');
robot.plot(q_traj, 'trail', 'r-', 'view', [-15 30], 'fps', 500); % 动画演示
title('机械臂运动动画');