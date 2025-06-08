%% 初始化
clear; clc; close all;
tic;

%% 加载机械臂模型和初始轨迹（示例结构）
run('P2P_free_motions.m'); % 假设该文件包含q_traj变量
original_traj = q_traj;    % Nx7矩阵，每个行向量为关节角度

%% 机械臂建模（与之前相同）
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'standard');
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0, 'standard');
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2, 'standard');
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0, 'standard');
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0, 'standard');
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0, 'standard');
robot = SerialLink(L, 'name', '7DOF-Arm');

%% 环境参数
obstacles = [-0.2 -0.2 -0.1 0.05;
             0 -0.2 -0.4 0.05;
             0.1 -0.3 0.1 0.05]; 
q_min = -pi*ones(1,7);
q_max = pi*ones(1,7);
delta = 0.1;        % RRT扩展步长
max_iter = 100;     % 每个修正点的最大尝试次数

%% 轨迹优化主算法
optimized_traj = original_traj(1,:); % 从起点开始
prev_valid = 1; % 前一个有效点索引

for k = 2:size(original_traj,1)
    current_point = original_traj(k,:);
    
    % 检查当前点是否直接可用
    if ~checkSingleConfigCollision(robot, current_point, obstacles)
        % 直接连接前一个有效点
        if ~checkPathCollision(robot, optimized_traj(end,:), current_point, obstacles, 0.05)
            optimized_traj = [optimized_traj; current_point];
            prev_valid = k;
            continue;
        end
    end
    
    % 需要修正的点进行RRT扩展
    [success, new_point] = rrt_extend(robot, optimized_traj(end,:), current_point,...
                                    obstacles, q_min, q_max, delta, max_iter);
    
    if success
        optimized_traj = [optimized_traj; new_point];
        prev_valid = k;
    else
        warning('点%d修正失败，保持原路径', k);
    end
end

% === 新增：轨迹平滑处理 === %
optimized_traj = smooth_trajectory(optimized_traj, robot, obstacles);

%% 结果验证与可视化
% 绘制优化前后轨迹对比
show_trajectory_comparison(robot, original_traj, optimized_traj, obstacles);
hold on;
fprintf('总优化时间: %.2f秒\n', toc);

%% === 新增轨迹平滑函数 === %
function smoothed = smooth_trajectory(traj, robot, obstacles)
    % 参数设置
    alpha = 0.3;   % 平滑系数（0-1，越大越平滑）
    beta = 0.1;    % 收缩系数（0-0.5）
    iterations = 50;  % 迭代次数
    
    smoothed = traj;
    n = size(traj,1);
    
    for iter = 1:iterations
        % 梯度下降平滑
        for i = 2:n-1
            % 原始点吸引项
            original_attraction = traj(i,:) - smoothed(i,:);
            
            % 平滑项（相邻点平均值）
            smooth_term = (smoothed(i-1,:) + smoothed(i+1,:)) / 2 - smoothed(i,:);
            
            % 障碍物排斥项
            [~, obs_dist] = get_nearest_obstacle(smoothed(i,:), robot, obstacles);
            obs_repulsion = -0.01 ./ (obs_dist.^2 + 0.001);  % 排斥力计算
            
            % 综合更新
            delta = alpha*smooth_term + beta*original_attraction + obs_repulsion;
            smoothed(i,:) = smoothed(i,:) + delta;
            
            % 保持关节限制
            smoothed(i,:) = max(min(smoothed(i,:)), robot.qlim(:,1)');
            smoothed(i,:) = min(smoothed(i,:), robot.qlim(:,2)');
        end
        
        % 碰撞检测修正
        for i = 2:n-1
            if checkSingleConfigCollision(robot, smoothed(i,:), obstacles)
                % 用前后安全点插值替代
                smoothed(i,:) = 0.5*smoothed(i-1,:) + 0.5*smoothed(i+1,:);
            end
        end
    end
end

%% === 新增辅助函数 === %
function [min_dist, nearest_dist] = get_nearest_obstacle(q, robot, obstacles)
    % 获取最近障碍物距离
    positions = getJointPositions(robot, q);
    nearest_dist = Inf;
    
    for j = 1:length(positions)
        for k = 1:size(obstacles,1)
            dist = norm(positions{j} - obstacles(k,1:3)) - obstacles(k,4);
            if dist < nearest_dist
                nearest_dist = dist;
            end
        end
    end
    min_dist = nearest_dist;
end

%% RRT扩展辅助函数
function [success, new_point] = rrt_extend(robot, q_start, q_target, obstacles, q_min, q_max, delta, max_iter)
    tree.nodes = q_start;
    tree.parent = 1;
    kdtree = KDTreeSearcher(tree.nodes);
    
    for iter = 1:max_iter
        % 目标偏置采样
        if rand < 0.3
            q_rand = q_target;
        else
            q_rand = q_min + (q_max - q_min).*rand(1,7);
        end
        
        % 最近邻搜索
        [idx_near, ~] = knnsearch(kdtree, q_rand);
        q_near = tree.nodes(idx_near,:);
        
        % 向随机点扩展
        direction = q_rand - q_near;
        dist = norm(direction);
        q_new = q_near + (direction / dist) * min(delta, dist);
        
        % 碰撞检测
        if ~checkPathCollision(robot, q_near, q_new, obstacles, 0.05)
            % 添加到树
            tree.nodes = [tree.nodes; q_new];
            tree.parent = [tree.parent; idx_near];
            kdtree = KDTreeSearcher(tree.nodes);
            
            % 检查是否可直达目标
            if ~checkPathCollision(robot, q_new, q_target, obstacles, 0.05)
                new_point = q_new;
                success = true;
                return;
            end
        end
    end
    success = false;
    new_point = q_start;
end

%% 轨迹可视化对比函数
%% 轨迹可视化对比函数（最终修正版）
function show_trajectory_comparison(robot, orig_traj, opt_traj, obstacles)
    % 障碍物绘制
    figure('Name','Trajectory Comparison');
    [X,Y,Z] = sphere(10);
    for i = 1:size(obstacles,1)
        surf(X*obstacles(i,4)+obstacles(i,1),...
             Y*obstacles(i,4)+obstacles(i,2),...
             Z*obstacles(i,4)+obstacles(i,3),...
             'FaceColor', [1 0.5 0], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
        hold on;
    end
    
    % 计算原始轨迹笛卡尔坐标
    n_pts = size(orig_traj,1);
    pos_orig = zeros(n_pts,3);
    for k = 1:n_pts
        T = robot.fkine(orig_traj(k,:));
        pos_orig(k,:) = T.t(1:3);
    end
    
    % 计算优化轨迹笛卡尔坐标
    n_pts_opt = size(opt_traj,1);
    pos_opt = zeros(n_pts_opt,3);
    for k = 1:n_pts_opt
        T = robot.fkine(opt_traj(k,:));
        pos_opt(k,:) = T.t(1:3);
    end
    
    % 绘制轨迹对比
    h1 = plot3(pos_orig(:,1), pos_orig(:,2), pos_orig(:,3),...
        'Color', [0 0.447 0.741], 'LineWidth', 1.5, 'LineStyle', '--');
    h2 = plot3(pos_opt(:,1), pos_opt(:,2), pos_opt(:,3),...
        'Color', [0.466 0.674 0.188], 'LineWidth', 2.2, 'LineStyle', '-');
    
    % 添加机器人末态显示
    robot.plot(opt_traj(end,:), 'workspace', [-0.5 0.5 -0.5 0.5 -0.5 0.5], 'delay',0);
    
    % 图形美化
    legend([h1 h2], {'Original Path', 'Optimized Path'}, 'Location','best');
    title('Trajectory Comparison');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal tight;
    grid on;
    view(-135, 30);  % 优化视角
    set(gca, 'FontSize', 11, 'FontName', 'Arial');
    camlight('headlight');
    lighting gouraud;
    hold off;
end

%% 碰撞检测函数（保持与之前相同）
function collision = checkPathCollision(robot, q1, q2, obstacles, step)
    n_steps = max(ceil(norm(q2 - q1)/step), 3);
    t = linspace(0, 1, n_steps);
    collision = false;
    
    for i = 1:n_steps
        q = q1*(1-t(i)) + q2*t(i);
        if checkSingleConfigCollision(robot, q, obstacles)
            collision = true;
            return;
        end
    end
end

function collision = checkSingleConfigCollision(robot, q, obstacles)
    positions = getJointPositions(robot, q);
    collision = false;
    
    for j = 1:length(positions)
        pos = positions{j};
        for k = 1:size(obstacles,1)
            if norm(pos - obstacles(k,1:3)) < obstacles(k,4)
                collision = true;
                return;
            end
        end
    end
end

function positions = getJointPositions(robot, q)
    positions = cell(1, robot.n+1);
    T = robot.base;
    positions{1} = T.t(1:3)';
    
    for i = 1:robot.n
        T = T * robot.links(i).A(q(i));
        positions{i+1} = T.t(1:3)';
    end
end