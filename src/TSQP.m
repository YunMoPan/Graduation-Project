%% 计时开始
tic;  % 启动计时器

%% 7DOF机械臂避障轨迹规划完整代码（顺序处理版）
clear; clc; close all;
s_max = 5;
run('P2P_free_motions.m');
mid_time = toc;
fprintf('加载自由轨迹，耗时 %.2f 秒\n', toc);

%% 1. 机械臂建模（基于DH参数）
% DH参数定义（标准型）
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'standard');
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0, 'standard');
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2, 'standard');
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0, 'standard');
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0, 'standard');
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0, 'standard');

robot = SerialLink(L, 'name', '7DOF-Arm');  % 创建串联机械臂对象

%% 2. 环境与轨迹初始化
q_min = -pi*ones(1,7);       % 关节下限（rad）
q_max = pi*ones(1,7);         % 关节上限（rad）

% 定义障碍物数量
if ~exist('obstacle_num', 'var') 
  obstacle_num = 4;
end

Obstacles = [-0.1 -0.2 -0.1 0.05;
             0 -0.2 -0.4 0.05;
            0.1 -0.3 0.1 0.05;
            0.1 -0.4 -0.1 0.05]; 

% 障碍物定义 [x,y,z,radius]
obstacles = Obstacles(1:obstacle_num, :); 

% 起始/目标位形（角度转弧度）
q_start = deg2rad([-31.20, -35.63, -28.07, 70.87, -24.63, -68.56, 52.63]);
q_goal  = deg2rad([-8, 3.9, 93.9, -101.7, -11.7, 3.9, -17.8]);

% 生成初始轨迹（使用Robotics Toolbox的jtraj函数）
Q_init = q_traj;

%% 3. 执行轨迹优化（顺序处理版）
Q_optimized = optimize_trajectory(robot, Q_init, obstacles, q_min, q_max);

%% 4. 轨迹碰撞验证
collision_free = true;
for i = 1:size(Q_optimized,1)
    [~, collision] = check_collision(robot, Q_optimized(i,:), obstacles);
    if collision
        fprintf('警告：点 %d 仍存在碰撞！\n', i);
        collision_free = false;
    end
end

if collision_free
    disp('== 轨迹已完全避障 ==');
else
    disp('== 存在未解决的碰撞 ==');
end

%% 5. 可视化验证（优化轨迹对比）
figure('Name','轨迹优化对比','Position',[100 100 560 420]);
% robot.plot(Q_init, 'fps',30,'trail','r-');  % 原始轨迹（红色）
[X,Y,Z] = sphere(20);  % 球体网格（20×20面）
[num, ~] = size(obstacles);
for obs_idx = 1:num
    obs = obstacles(obs_idx,:);
    surf(X*obs(4)+obs(1), Y*obs(4)+obs(2), Z*obs(4)+obs(3),...
        'FaceColor',[0.9 0.2 0.2],'FaceAlpha',0.3,'EdgeColor','none');
    hold on;
end
robot.plot(Q_optimized, 'fps',30, 'view', [-45 30], 'trail','r-');  % 优化轨迹（红色）

%% 计算总耗时
elapsed_time = toc;  % 停止计时器
fprintf('TRRT算法总运行时间: %.2f 秒\n', elapsed_time);
fprintf('优化完成，耗时 %.2f 秒\n', toc-mid_time);
% path = Q_optimized;
% run('human_draw.m');

% 绘制障碍物（半透明球体）
% [X,Y,Z] = sphere(20);  % 球体网格（20×20面）
% for obs_idx = 1:size(obstacles,1)
%     obs = obstacles(obs_idx,:);
%     surf(X*obs(4)+obs(1), Y*obs(4)+obs(2), Z*obs(4)+obs(3),...
%         'FaceColor',[0.9 0.2 0.2],'FaceAlpha',0.3,'EdgeColor','none');
% end
% axis tight; view(3); grid on;
% xlabel('X轴 (m)'); ylabel('Y轴 (m)'); zlabel('Z轴 (m)');
% title('原始轨迹（红）与优化轨迹（蓝）对比');

%% 核心函数：轨迹优化（顺序处理版）
function Q_opt = optimize_trajectory(robot, Q_init, obstacles, q_min, q_max)
    % 优化参数设置
    safe_dist = 0.05;    % 安全距离（障碍物表面到机械臂的最小允许距离）
    w_collision = 1000;  % 碰撞代价权重（惩罚碰撞风险）
    w_smooth = 100;      % 平滑项权重（避免轨迹突变）
    max_iter = 15;       % 最大迭代次数
    
    Q_opt = Q_init;      % 初始化优化轨迹为原始轨迹
    n_points = size(Q_opt,1);  % 轨迹时间步数（120）
    
    % 创建优化选项（使用SQP算法）
    options = optimoptions('fmincon',...
        'Algorithm','sqp',...
        'Display','off',...      % 关闭优化过程显示
        'MaxIterations',50,...   % 单次优化最大迭代次数
        'StepTolerance',1e-6);   % 步长容差
    
    % 顺序优化轨迹点（替换原parfor为普通for循环）
    for iter = 1:max_iter
        collision_flags = false(n_points,1);  % 标记碰撞点
        
        % 遍历轨迹点（忽略首尾点，避免边界问题）
        for i = 2:n_points-1
            q_current = Q_opt(i,:);  % 当前轨迹点关节角
            
            % 检查当前点是否碰撞
            [~, collision] = check_collision(robot, q_current, obstacles);
            if collision
                collision_flags(i) = true;  % 标记为碰撞点
                
                % 定义优化问题：最小化轨迹偏离和不平滑度
                cost_func = @(q) trajectory_cost(q, Q_init(i,:), Q_opt(i-1,:), Q_opt(i+1,:), w_collision, w_smooth);
                constr_func = @(q) collision_constraint(robot, q, obstacles, safe_dist);
                
                % 执行优化（使用fmincon求解）
                q_opt = fmincon(cost_func, q_current,...
                    [],[],[],[],...          % 无线性约束
                    q_min, q_max,...         % 关节角度限制
                    constr_func,...          % 碰撞约束
                    options);                % 优化选项
                
                Q_opt(i,:) = q_opt;  % 更新优化轨迹点
            end
        end
        
        % 对碰撞点附近轨迹应用平滑滤波
        Q_opt = smooth_trajectory(Q_opt, collision_flags);
        
        % 显示迭代进度
        fprintf('迭代 %d/%d, 剩余碰撞点数量: %d\n',...
            iter, max_iter, sum(collision_flags));
        
        % 提前终止条件（无碰撞点时退出循环）
        if ~any(collision_flags)
            break;
        end
    end
    
    % 最终全局平滑处理（高斯滤波）
    Q_opt = smoothdata(Q_opt, 'gaussian', 7);  % 7点高斯窗口平滑
end

%% 辅助函数1：碰撞检测（计算机械臂与障碍物的最小距离）
function [min_dist, is_collision] = check_collision(robot, q, obstacles)
    min_dist = Inf;
    is_collision = false;
    
    prev_pos = [0 0 0]; % 基座原点作为起始位置
    
    for link_idx = 1:robot.n
        T = robot.A(link_idx, q).T; % 获取当前连杆变换矩阵
        curr_pos = T(1:3,4)';       % 当前连杆末端位置
        
        for obs_idx = 1:size(obstacles,1)
            obs = obstacles(obs_idx,:);
            [dist, ~] = point_to_line_distance(obs(1:3), prev_pos, curr_pos);
            effective_dist = dist - obs(4);
            
            if effective_dist < min_dist
                min_dist = effective_dist;
            end
            if effective_dist < 0
                is_collision = true;
            end
        end
        prev_pos = curr_pos; % 更新为下一连杆起点
    end
end

%% 辅助函数2：点到线段的距离计算
function [distance, t] = point_to_line_distance(p, a, b)
    ap = p - a;       % 向量AP
    ab = b - a;       % 向量AB
    t = dot(ap, ab) / dot(ab, ab);  % 投影参数t
    t = max(0, min(1, t));          % 限制t在[0,1]内（线段范围内）
    nearest_point = a + t * ab;     % 线段上最近点
    distance = norm(p - nearest_point);  % 点到线段的距离
end

%% 辅助函数3：轨迹代价函数（优化目标）
function cost = trajectory_cost(q, q_init, q_prev, q_next, w_col, w_smo)
    % 轨迹偏离代价（与原始轨迹的差异）
    deviation_cost = norm(q - q_init)^2;
    
    % 平滑代价（与前后点的差异，避免突变）
    smooth_cost = norm(q - q_prev)^2 + norm(q_next - q)^2;
    
    % 总代价（加权和）
    cost = deviation_cost + w_smo * smooth_cost;
end

%% 辅助函数4：碰撞约束函数（确保最小距离≥安全阈值）
function [c, ceq] = collision_constraint(robot, q, obstacles, safe_dist)
    [min_dist, ~] = check_collision(robot, q, obstacles);  % 计算当前位形的最小距离
    c = safe_dist - min_dist;  % 不等式约束：min_dist ≥ safe_dist → c ≤ 0
    ceq = [];                  % 无等式约束
end

%% 辅助函数5：轨迹平滑（局部三点平均）
function Q_smooth = smooth_trajectory(Q, flags)
    Q_smooth = Q;
    window = [0.25; 0.5; 0.25];  % 三点平滑窗（权重：前点25%，当前点50%，后点25%）
    
    % 仅对标记为碰撞的点附近进行平滑
    for i = 2:size(Q,1)-1
        if flags(i)
            Q_smooth(i,:) = window(1)*Q(i-1,:) + window(2)*Q(i,:) + window(3)*Q(i+1,:);
        end
    end
end