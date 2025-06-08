numericArray = path;

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
path = joint_angles;

%% 生成速度、加速度、急动度曲线（固定时间间隔 dt=0.1 秒）
% ====================== 参数设置（固定 dt=0.1 秒） ======================
dt = 0.1;                     % 固定时间间隔（秒）
N = size(path, 1);            % 轨迹点数
t = (0:N-1) * dt;             % 时间向量（与轨迹点一一对应）

% ====================== 计算各阶导数 ======================
% 速度（qd）: 相邻轨迹点的差分除以 dt，形状为 (N-1)×7
qd = diff(path, 1, 1) ./ dt;   % 等价于 diff(path)/dt（显式指定差分维度）

% 加速度（qdd）: 速度的差分除以 dt，形状为 (N-2)×7
qdd = diff(qd, 1, 1) ./ dt;

% 急动度（qddd）: 加速度的差分除以 dt，形状为 (N-3)×7
qddd = diff(qdd, 1, 1) ./ dt;

% 急动度范数（每个时间点的急动度向量 L2 范数）
jerk_norm = sqrt(sum(qddd.^2, 2));  % 按列求和，每行对应一个时间点的范数

% ====================== 绘制速度曲线 ======================
figure('Name', '关节速度曲线', 'Position', [100 100 1200 800]);
for i = 1:7
    subplot(4, 2, i);  % 4行2列子图（第7个关节占用最后一个子图）
    % 速度对应的时间点：t(1:end-1)（共 N-1 个时间点，对应 N-1 个速度值）
    plot(t(1:end-1), qd(:,i), 'b-', 'LineWidth', 1.5);  
    title(sprintf('关节 %d 速度曲线', i));
    xlabel('时间 (s)');
    ylabel('速度 (rad/s)');
    grid on;
    % 自动调整纵坐标范围（预留 10% 空白）
    y_min = min(qd(:,i)) - 0.1*abs(min(qd(:,i)));
    y_max = max(qd(:,i)) + 0.1*abs(max(qd(:,i)));
    ylim([y_min, y_max]);
end
sgtitle('各关节速度随时间变化曲线');  % 总标题

% ====================== 绘制急动度范数曲线 ======================
figure('Name', '急动度范数曲线', 'Position', [100 100 800 500]);
% 急动度对应的时间点：t(1:end-3)（共 N-3 个时间点，对应 N-3 个急动度值）
t_jerk = t(1:end-3);  
plot(t_jerk, jerk_norm, 'r-', 'LineWidth', 1.5);
title('急动度范数随时间变化曲线');
xlabel('时间 (s)');
ylabel('急动度范数 (rad/s³)');
grid on;
ylim([0, max(jerk_norm) + 0.1*max(jerk_norm)]);  % 预留 10% 空白

% ---------------------- 标注最大值点 ----------------------
if ~isempty(jerk_norm)  % 防止轨迹点数不足导致空数组
    [max_jerk, idx_max] = max(jerk_norm);
    t_max = t_jerk(idx_max);
    
    % 数据坐标标注（推荐）
    text(t_max, max_jerk, ...
        sprintf('Max: %.3f\n(t=%.2f s)', max_jerk, t_max), ...
        'HorizontalAlignment', 'right', ...  % 文本在点的左侧
        'VerticalAlignment', 'middle', ...
        'Color', 'blue', ...
        'FontSize', 10, ...
        'BackgroundColor', [1 0.95 0.9],...  % 淡红色背景
        'EdgeColor', 'none');             % 无边框
    
    % 添加标记点（红色五角星）
    hold on;
    plot(t_max, max_jerk, 'kp', 'MarkerSize', 8, 'MarkerFaceColor', 'red');  % 红色五角星
    hold off;
end