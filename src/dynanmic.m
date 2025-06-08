%% 7DOF机械臂避障轨迹规划 (带动态障碍物)
clear; clc; close all;
tic;

%% 1. 机械臂建模（基于DH参数）
% 简化DH参数定义
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'standard');
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0, 'standard');
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2, 'standard');
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0, 'standard');
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0, 'standard');
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0, 'standard');

robot = SerialLink(L, 'name', '7DOF-Arm');

%% 2. 环境初始化
q_min = -pi*ones(1,7);       % 关节下限（rad）
q_max = pi*ones(1,7);         % 关节上限（rad）

% 定义障碍物数量
obstacle_num = 4;

% 起始/目标位形
q_start = deg2rad([-31.20, -35.63, -28.07, 70.87, -24.63, -68.56, 52.63]);
q_goal  = deg2rad([-8, 3.9, 93.9, -101.7, -11.7, 3.9, -17.8]);

%% 3. 生成动态障碍物轨迹
fprintf('生成动态障碍物轨迹...\n');
total_time = 6;  % 总时间 (秒)
dt = 0.05;       % 时间步长 (秒)
t_steps = 0:dt:total_time;
num_steps = length(t_steps);

% 初始障碍物位置
initial_obstacles = [-0.1 -0.2 -0.1 0.05;
                     0   -0.2 -0.4 0.05;
                     0.1 -0.3  0.1 0.05;
                     0.1 -0.4 -0.1 0.05];

% 生成动态障碍物轨迹
dynamic_obstacles = generate_dynamic_obstacles(initial_obstacles, total_time, dt);

%% 4. 生成初始轨迹
s_max = 5;
run('P2P_free_motions.m');
num_points = 120;  % 轨迹点数
Q_init = q_traj;

% 轨迹时间向量 (与动态障碍物时间同步)
traj_times = linspace(0, total_time, num_points);

%% 5. 优化方法选择 (考虑动态障碍物)
fprintf('===== 使用梯度优化轨迹 (动态障碍物) =====\n');

% 梯度优化参数
opt_params = struct();
opt_params.safe_dist = 0.05;      % 安全距离阈值
opt_params.lambda_smooth = 0.3;   % 平滑性权重
opt_params.max_iter = 20;         % 迭代次数 (增加迭代次数以适应动态障碍物)
opt_params.learning_rate = 0.02;  % 学习率
opt_params.min_improve = 1e-4;    % 最小改进阈值
opt_params.total_time = total_time;
opt_params.dt = dt;

% 执行梯度优化 (考虑动态障碍物)
Q_optimized = dynamic_trajectory_optimization(robot, Q_init, traj_times, ...
                    dynamic_obstacles, q_min, q_max, opt_params);

%% 6. 动态可视化结果
visualize_dynamic_results(robot, Q_optimized, traj_times, dynamic_obstacles, q_start, q_goal);

%% 7. 性能评估
fprintf('总耗时: %.2f 秒\n', toc);

% 轨迹相似度详细评估
diff = Q_optimized - Q_init;
rmsd = sqrt(mean(diff(:).^2));
max_diff = max(abs(diff(:)));
avg_diff = mean(abs(diff(:)));

fprintf('\n===== 轨迹相似度评估 =====\n');
fprintf('RMSD: %.4f rad (%.1f°)\n', rmsd, rad2deg(rmsd));
fprintf('最大偏差: %.4f rad (%.1f°)\n', max_diff, rad2deg(max_diff));
fprintf('平均偏差: %.4f rad (%.1f°)\n', avg_diff, rad2deg(avg_diff));

% 可视化偏差分布
figure('Name', '关节角度偏差分布');
histogram(rad2deg(diff(:)), 50);
title('关节角度偏差分布');
xlabel('偏差 (°)');
ylabel('频数');
grid on;

%% ========== 辅助函数 ==========

% 生成动态障碍物轨迹 (修复维度问题)
function dynamic_obstacles = generate_dynamic_obstacles(initial_obstacles, total_time, dt)
    num_obstacles = size(initial_obstacles, 1);
    time_steps = 0:dt:total_time;
    num_steps = length(time_steps);
    
    % 初始化动态障碍物数组 (4维: 障碍物索引 x 时间步 x 坐标/半径)
    dynamic_obstacles = zeros(num_obstacles, num_steps, 4);
    
    % 设置初始位置
    for i = 1:num_obstacles
        dynamic_obstacles(i, 1, :) = initial_obstacles(i, :);
    end
    
    % 生成随机移动轨迹
    max_speed = 0.1;  % 最大移动速度 (单位/秒)
    velocities = zeros(num_obstacles, 3); % 存储每个障碍物的速度
    
    % 初始化速度
    for i = 1:num_obstacles
        velocities(i, :) = max_speed * (2*rand(1,3) - 1);
    end
    
    for t = 2:num_steps
        for i = 1:num_obstacles
            % 随机速度变化
            velocity_change = max_speed * (2*rand(1,3) - 1) * 0.5;
            velocities(i, :) = velocities(i, :) + velocity_change;
            
            % 限制最大速度
            speed = norm(velocities(i, :));
            if speed > max_speed
                velocities(i, :) = velocities(i, :) * (max_speed / speed);
            end
            
            % 获取上一个位置
            prev_pos = squeeze(dynamic_obstacles(i, t-1, 1:3))';
            
            % 更新位置
            new_pos = prev_pos + velocities(i, :) * dt;
            
            % 边界约束 (可选)
            % new_pos = min(max(new_pos, [-0.5, -0.8, -0.8]), [0.5, 0.2, 0.4]);
            
            % 保存新位置 (半径保持不变)
            dynamic_obstacles(i, t, 1:3) = new_pos;
            dynamic_obstacles(i, t, 4) = dynamic_obstacles(i, t-1, 4);
        end
    end
end

% 动态轨迹优化函数
function Q_opt = dynamic_trajectory_optimization(robot, Q_init, traj_times, ...
                          dynamic_obstacles, q_min, q_max, params)
    fprintf('执行动态轨迹优化...\n');
    
    Q_opt = Q_init;
    n = size(Q_opt, 1);
    prev_cost = inf;
    num_obstacles = size(dynamic_obstacles, 1);
    num_time_steps = size(dynamic_obstacles, 2);
    
    % 计算初始轨迹安全成本
    init_safety_cost = 0;
    for i = 1:n
        t_idx = max(1, min(round(i/n * num_time_steps), num_time_steps));
        current_obstacles = squeeze(dynamic_obstacles(:, t_idx, :))';
        [min_dist, ~] = check_collision(robot, Q_opt(i,:), current_obstacles);
        init_safety_cost = init_safety_cost + max(0, params.safe_dist - min_dist);
    end
    init_safety_cost = init_safety_cost / n;
    
    % 批量梯度优化
    for iter = 1:params.max_iter
        total_improvement = 0;
        current_lr = params.learning_rate * (1 - iter/params.max_iter)^0.5;
        
        % 计算全轨迹梯度
        grad_total = zeros(size(Q_opt));
        for i = 2:n-1
            t_idx = max(1, min(round(i/n * num_time_steps), num_time_steps));
            current_obstacles = squeeze(dynamic_obstacles(:, t_idx, :))';
            
            [~, grad] = dynamic_point_cost(robot, Q_opt, i, current_obstacles, params);
            grad_total(i,:) = grad;
        end
        
        % 应用批量更新
        Q_new = Q_opt - current_lr * grad_total;
        
        % 应用关节限制
        Q_new = max(min(Q_new, q_max), q_min);
        
        % 计算新成本
        new_cost = 0;
        for i = 1:n
            t_idx = max(1, min(round(i/n * num_time_steps), num_time_steps));
            current_obstacles = squeeze(dynamic_obstacles(:, t_idx, :))';
            [cost_i, ~] = dynamic_point_cost(robot, Q_new, i, current_obstacles, params);
            new_cost = new_cost + cost_i;
        end
        new_cost = new_cost / n;
        
        % 计算旧成本
        old_cost = 0;
        for i = 1:n
            t_idx = max(1, min(round(i/n * num_time_steps), num_time_steps));
            current_obstacles = squeeze(dynamic_obstacles(:, t_idx, :))';
            [cost_i, ~] = dynamic_point_cost(robot, Q_opt, i, current_obstacles, params);
            old_cost = old_cost + cost_i;
        end
        old_cost = old_cost / n;
        
        % 接受改进
        if new_cost < old_cost
            Q_opt = Q_new;
            total_improvement = old_cost - new_cost;
        end
        
        % 计算整个轨迹的安全成本
        safety_cost = 0;
        min_safety = inf;
        for i = 1:n
            t_idx = max(1, min(round(i/n * num_time_steps), num_time_steps));
            current_obstacles = squeeze(dynamic_obstacles(:, t_idx, :))';
            [min_dist, ~] = check_collision(robot, Q_opt(i,:), current_obstacles);
            safety_cost = safety_cost + max(0, params.safe_dist - min_dist);
            if min_dist < min_safety
                min_safety = min_dist;
            end
        end
        safety_cost = safety_cost / n;
        
        fprintf('迭代 %d/%d: 安全成本=%.4f, 最小安全距离=%.4f, 改进=%.4f\n', ...
                iter, params.max_iter, safety_cost, min_safety, total_improvement);
        
        % 检查收敛
        if total_improvement < params.min_improve && iter > 5
            fprintf('提前收敛于迭代 %d\n', iter);
            break;
        end
        
        prev_cost = safety_cost;
    end
    
    % 最终平滑
    Q_opt = smoothdata(Q_opt, 'gaussian', 7);
    
    % 计算最终安全指标
    final_safety = 0;
    min_safety = inf;
    for i = 1:size(Q_opt,1)
        t_idx = max(1, min(round(i/n * num_time_steps), num_time_steps));
        current_obstacles = squeeze(dynamic_obstacles(:, t_idx, :))';
        [min_dist, ~] = check_collision(robot, Q_opt(i,:), current_obstacles);
        final_safety = final_safety + min_dist;
        if min_dist < min_safety
            min_safety = min_dist;
        end
    end
    avg_safety = final_safety / size(Q_opt,1);
    
    fprintf('优化完成: 初始安全成本=%.4f, 最终平均安全距离=%.4f, 最小安全距离=%.4f\n', ...
            init_safety_cost, avg_safety, min_safety);
end

% 动态点成本计算
function [cost, grad] = dynamic_point_cost(robot, Q, idx, obstacles, params)
    q = Q(idx,:);
    
    % 计算当前点与障碍物的最小距离
    [min_dist, ~] = check_collision(robot, q, obstacles);
    
    % 计算安全成本（线性惩罚）
    safety_cost = max(0, params.safe_dist - min_dist);
    
    % 计算平滑成本（与相邻点的差异）
    if idx == 1
        smooth_cost = sum((Q(idx,:) - Q(idx+1,:)).^2);
        grad_smooth = 2*(Q(idx,:) - Q(idx+1,:));
    elseif idx == size(Q,1)
        smooth_cost = sum((Q(idx,:) - Q(idx-1,:)).^2);
        grad_smooth = 2*(Q(idx,:) - Q(idx-1,:));
    else
        smooth_cost = 0.5*sum((Q(idx,:) - Q(idx-1,:)).^2) + ...
                      0.5*sum((Q(idx,:) - Q(idx+1,:)).^2);
        grad_smooth = (Q(idx,:) - Q(idx-1,:)) + (Q(idx,:) - Q(idx+1,:));
    end
    
    % 总成本
    cost = safety_cost + params.lambda_smooth * smooth_cost;
    
    % 数值梯度计算
    epsilon = 1e-6;
    grad = zeros(1,7);
    for j = 1:7
        q_plus = q;
        q_plus(j) = q_plus(j) + epsilon;
        [min_dist_plus, ~] = check_collision(robot, q_plus, obstacles);
        safety_cost_plus = max(0, params.safe_dist - min_dist_plus);
        
        q_minus = q;
        q_minus(j) = q_minus(j) - epsilon;
        [min_dist_minus, ~] = check_collision(robot, q_minus, obstacles);
        safety_cost_minus = max(0, params.safe_dist - min_dist_minus);
        
        grad(j) = (safety_cost_plus - safety_cost_minus) / (2*epsilon);
    end
    
    % 添加平滑梯度
    grad = grad + params.lambda_smooth * grad_smooth;
end

% 动态结果可视化
function visualize_dynamic_results(robot, Q, traj_times, dynamic_obstacles, q_start, q_goal)
    fprintf('创建动态可视化...\n');
    
    % 创建图形窗口
    fig = figure('Name','动态避障轨迹','Position',[100 100 800 600]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, [-70 30]);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, '7DOF机械臂动态避障');
    
    % 设置坐标轴范围
    axis(ax, [-0.5 0.5 -0.8 0.2 -0.8 0.4]);
    
    % 绘制起始和目标点
    T_start = robot.fkine(q_start);
    T_goal = robot.fkine(q_goal);
    start_pt = plot3(ax, T_start.t(1), T_start.t(2), T_start.t(3), 'go', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'g');
    goal_pt = plot3(ax, T_goal.t(1), T_goal.t(2), T_goal.t(3), 'ro', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % 创建障碍物球体
    num_obstacles = size(dynamic_obstacles, 1);
    num_obs_time_steps = size(dynamic_obstacles, 2);  % 障碍物轨迹时间步数
    obstacle_handles = gobjects(num_obstacles, 1);
    [X,Y,Z] = sphere(10);  % 简化球体网格
    
    % 轨迹线
    traj_handle = plot3(ax, nan, nan, nan, 'b-', 'LineWidth', 1.5);
    
    % 创建障碍物轨迹线
    obstacle_traj_handles = gobjects(num_obstacles, 1);
    colors = lines(num_obstacles);
    for i = 1:num_obstacles
        obstacle_traj_handles(i) = plot3(ax, nan, nan, nan, ...
            'Color', [colors(i,:) 0.3], 'LineWidth', 1);
    end
    
    % 初始绘制
    for i = 1:num_obstacles
        obs = squeeze(dynamic_obstacles(i, 1, :));
        obstacle_handles(i) = surf(ax, X*obs(4)+obs(1), Y*obs(4)+obs(2), Z*obs(4)+obs(3), ...
            'FaceColor', colors(i,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    end
    pause(10);
    % 绘制初始机械臂位置
    robot.plot(q_start, 'nobase', 'noname', 'nowrist', 'noshadow');
    
    % 添加图例
    legend(ax, [start_pt, goal_pt, traj_handle, obstacle_handles(1)], ...
        {'起点', '终点', '机械臂轨迹', '障碍物'}, 'Location', 'best');
    
    % 添加光照
    light('Position',[1 1 1],'Style','infinite');
    lighting gouraud;
    
    % 动画参数
    fps = 20;
    delay = 1/fps;
    
    % 存储轨迹数据
    num_traj_points = size(Q, 1);
    ee_positions = zeros(num_traj_points, 3);
    obstacle_positions = zeros(num_traj_points, num_obstacles, 3);
    
    % 计算机械臂轨迹时间与障碍物轨迹时间的映射关系
    total_traj_time = traj_times(end);
    total_obs_time = (num_obs_time_steps - 1) * (total_traj_time / (num_traj_points - 1));
    
    % 预计算轨迹
    for i = 1:num_traj_points
        % 计算当前时间点
        current_time = traj_times(i);
        
        % 计算对应的障碍物时间索引（线性映射）
        obs_time_idx = round(current_time / total_obs_time * (num_obs_time_steps - 1)) + 1;
        obs_time_idx = max(1, min(obs_time_idx, num_obs_time_steps));
        
        % 计算机械臂末端位置
        T = robot.fkine(Q(i,:));
        ee_positions(i,:) = T.t';
        
        % 获取当前时间对应的障碍物位置
        for j = 1:num_obstacles
            obs = squeeze(dynamic_obstacles(j, obs_time_idx, :));
            obstacle_positions(i, j, :) = obs(1:3);
        end
    end
    
    % 动画循环
    for i = 1:num_traj_points
        % 更新机械臂位置
        robot.plot(Q(i,:), 'fps', fps, 'nobase', 'noname', 'nowrist', 'noshadow');
        
        % 更新末端轨迹
        set(traj_handle, 'XData', ee_positions(1:i,1), ...
                         'YData', ee_positions(1:i,2), ...
                         'ZData', ee_positions(1:i,3));
        
        % 更新障碍物位置和轨迹
        for j = 1:num_obstacles
            % 获取当前障碍物在当前时间点的位置
            current_obs_time = round(traj_times(i) / total_obs_time * (num_obs_time_steps - 1)) + 1;
            current_obs_time = max(1, min(current_obs_time, num_obs_time_steps));
            obs = squeeze(dynamic_obstacles(j, current_obs_time, :));
            r = obs(4);
            
            % 更新障碍物位置
            set(obstacle_handles(j), 'XData', X*r + obs(1), ...
                                     'YData', Y*r + obs(2), ...
                                     'ZData', Z*r + obs(3));
            
            % 更新障碍物轨迹
            obs_traj = squeeze(obstacle_positions(1:i, j, :));
            if size(obs_traj, 1) > 0 && size(obs_traj, 2) >= 3  % 确保数据维度正确
                set(obstacle_traj_handles(j), 'XData', obs_traj(:,1), ...
                                              'YData', obs_traj(:,2), ...
                                              'ZData', obs_traj(:,3));
            end
        end
        
        % 添加当前时间标题
        title(ax, sprintf('7DOF机械臂动态避障 (时间: %.2f秒)', traj_times(i)));
        
        % 暂停以创建动画效果
        pause(delay);
    end
    
    % 添加最终标题
    title(ax, '7DOF机械臂动态避障');
    
    % 计算并显示最小安全距离
    min_safety = Inf;
    for i = 1:num_traj_points
        % 计算对应的障碍物时间索引
        obs_time_idx = round(traj_times(i) / total_obs_time * (num_obs_time_steps - 1)) + 1;
        obs_time_idx = max(1, min(obs_time_idx, num_obs_time_steps));
        
        current_obstacles = squeeze(dynamic_obstacles(:, obs_time_idx, :))';
        [min_dist, ~] = check_collision(robot, Q(i,:), current_obstacles);
        if min_dist < min_safety
            min_safety = min_dist;
        end
    end
    fprintf('轨迹最小安全距离: %.4f m\n', min_safety);
end

%% 碰撞检测函数
function [min_dist, is_collision] = check_collision(robot, q, obstacles)
    min_dist = Inf;
    is_collision = false;
    
    % 获取所有连杆的位置
    positions = zeros(8, 3);
    positions(1,:) = [0, 0, 0];
    
    % 计算每个关节的位置
    for i = 1:7
        T = robot.A(i, q).T;
        positions(i+1,:) = T(1:3,4)';
    end
    
    % 检查每个连杆与障碍物的距离
    for link_idx = 1:7
        a = positions(link_idx, :);
        b = positions(link_idx+1, :);
        
        for obs_idx = 1:size(obstacles,1)
            obs = obstacles(obs_idx,1:3);
            radius = obstacles(obs_idx,4);
            [dist, ~] = point_to_line_distance(obs, a, b);
            effective_dist = dist - radius;
            
            if effective_dist < min_dist
                min_dist = effective_dist;
            end
            if effective_dist < 0
                is_collision = true;
            end
        end
    end
end

%% 点到线段的距离计算
function [distance, t] = point_to_line_distance(p, a, b)
    ap = p - a;
    ab = b - a;
    t = dot(ap, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    nearest_point = a + t * ab;
    distance = norm(p - nearest_point);
end