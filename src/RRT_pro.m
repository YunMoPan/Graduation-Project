%% 计时开始
tic;  % 启动计时器

%% 机械臂定义（保持不变）
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'standard');
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0, 'standard');
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2, 'standard');
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0, 'standard');
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0, 'standard');
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0, 'standard');
robot = SerialLink(L, 'name', '7DOF-Arm');

%% 参数设置（保持不变）
q_min = -pi*ones(1,7);
q_max = pi*ones(1,7);
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
q_start = deg2rad([-31.20386081, -35.6289004, -28.06637695, 70.8710050038914, -24.6309139, -68.56067049, 52.6344433733276]);
q_goal = deg2rad([-8, 3.9, 93.9, -101.7, -11.7, 3.9, -17.8]);
max_iter = 5000; delta = 0.3; goal_tolerance = 0.1; prob_goal = 0.2; gamma = 3.5; d = 7; collision_step = 0.05;

%% RRT初始化（保持不变）
node_matrix = q_start;  % 直接存储所有节点（n×7矩阵，每行一个位形）
tree.parent = 0;
tree.cost = 0;
goal_reached = false;
best_goal_idx = 0;

%% 新增：初始化前一次代价变量（关键修改）
prev_goal_cost = [];  % 初始化为空

%% 可视化设置（保持不变）
figure('Position', [440, 300, 560, 420]);
robot.plot(q_start); hold on;
% plot3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 'ro', 'MarkerSize', 10);
% 设置颜色映射
% 使用Okabe-Ito经典配色（直接定义RGB值）
color_map = [0.90, 0.60, 0.00;   % 橙色
             0.35, 0.70, 0.90;   % 浅蓝
             0.80, 0.40, 0.00;   % 棕红
             0.30, 0.65, 0.30];  % 深绿

% 绘制障碍物球体
for i = 1:size(obstacles, 1)
    [x, y, z] = sphere(50);  % 生成球面网格（分辨率50）
    
    % 缩放和平移球体
    x = x * obstacles(i,4) + obstacles(i,1);
    y = y * obstacles(i,4) + obstacles(i,2);
    z = z * obstacles(i,4) + obstacles(i,3);
    
    % 绘制带光照和透明度的球体
    surf(x, y, z, 'FaceColor', color_map(i,1:3), ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.6, ...   % 透明度
        'FaceLighting', 'gouraud', ...
        'DiffuseStrength', 0.8, ...
        'SpecularStrength', 0.3);
end

% 添加光照和背景效果
light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud;       % 光滑光照模式
material dull;          % 材质反射属性
axis equal vis3d;       % 保持三维坐标系比例
view(70, 30);           % 调整视角
h_viz = plot3(nan, nan, nan, 'b.', 'MarkerSize', 5); % 预创建可视化对象
drawnow;

%% 主循环（修复 `persistent` 错误）
for iter = 1:max_iter
    % 1. 随机采样（保持不变）
    if rand < prob_goal
        q_rand = q_goal;
    else
        q_rand = q_min + (q_max - q_min).*rand(1,7);
    end
    
    % 2. 最近邻搜索（保持不变）
    [idx_near, ~] = knnsearch(node_matrix, q_rand, 'K', 1);  % 正确：第一个参数是数据矩阵
    q_near = node_matrix(idx_near, :);
    
    % 3. 生成新节点（保持不变）
    direction = q_rand - q_near;
    dist = norm(direction);
    q_new = q_near + (direction / max(dist, 1e-6)) * min(delta, dist);
    
    % 4. 关节限制检查（保持不变）
    if any(q_new < q_min - 1e-6) || any(q_new > q_max + 1e-6), continue; end
    
    % 5. 路径碰撞检测（保持不变）
    if ~checkPathCollision(robot, q_near, q_new, obstacles, collision_step)
        % ==== RRT*核心逻辑（保持不变）====
        n_nodes = size(node_matrix, 1);
        r = gamma * (log(n_nodes)/n_nodes)^(1/d);  % 邻域半径
        
        % 6. 邻域搜索（保持不变）
        [near_indices, ~] = rangesearch(node_matrix, q_new, r);  % 正确：第一个参数是数据矩阵
        near_indices = near_indices{1};
        if isempty(near_indices), near_indices = idx_near; end  % 至少包含最近节点
        
        % 7. 寻找最优父节点（保持不变）
        min_cost = Inf; best_parent = 0;
        for k = 1:length(near_indices)
            parent_idx = near_indices(k);
            parent_q = node_matrix(parent_idx, :);
            if ~checkPathCollision(robot, parent_q, q_new, obstacles, collision_step)
                candidate_cost = tree.cost(parent_idx) + norm(q_new - parent_q);
                if candidate_cost < min_cost
                    min_cost = candidate_cost;
                    best_parent = parent_idx;
                end
            end
        end
        
        % 8. 添加新节点（保持不变）
        if best_parent == 0, continue; end
        node_matrix = [node_matrix; q_new];  % 追加新节点到矩阵
        tree.parent = [tree.parent; best_parent];
        tree.cost = [tree.cost; min_cost];
        
        % 9. 邻域节点重连（保持不变）
        for k = 1:length(near_indices)
            target_idx = near_indices(k);
            target_q = node_matrix(target_idx, :);
            new_cost = min_cost + norm(target_q - q_new);
            if new_cost < tree.cost(target_idx) && ...
               ~checkPathCollision(robot, q_new, target_q, obstacles, collision_step)
                tree.parent(target_idx) = n_nodes + 1;
                tree.cost(target_idx) = new_cost;
            end
        end
        
        % ==== 可视化（保持不变）====
        T_new = robot.fkine(q_new);
        set(h_viz, 'XData', [get(h_viz,'XData') T_new.t(1)],...
                  'YData', [get(h_viz,'YData') T_new.t(2)],...
                  'ZData', [get(h_viz,'ZData') T_new.t(3)]);
        if mod(iter, 50) == 0, drawnow limitrate; end
        
        % 目标检测与代价稳定性判断（关键修改：移除 persistent）
        if norm(q_new - q_goal) < goal_tolerance
            current_goal_cost = min_cost;  % 当前目标节点代价
            
            % 首次记录或稳定性检查（使用脚本变量）
            if isempty(prev_goal_cost)
                prev_goal_cost = current_goal_cost;  % 首次记录代价
            else
                % 连续两次代价稳定（容差范围内）则终止
                if abs(current_goal_cost - prev_goal_cost) < 1e-6  
                    disp('目标代价稳定，提前停止搜索');
                    goal_reached = true;
                    best_goal_idx = size(node_matrix, 1);
                    break;  % 跳出主循环
                end
                prev_goal_cost = current_goal_cost;  % 更新前一次代价
            end
            
            disp(['迭代', num2str(iter), '：目标到达！当前代价', num2str(current_goal_cost)]);
        end
    end
end

%% 路径回溯与可视化（保持不变）
%% 路径回溯与可视化（关键修改：增加轨迹插值）
if goal_reached
    goal_dist = sqrt(sum((node_matrix - repmat(q_goal, size(node_matrix,1), 1)).^2, 2));  % 手动计算欧氏距离
    [~, best_idx] = min(goal_dist(goal_dist < goal_tolerance));
    path = []; current = best_idx;
    while current ~= 0
        path = [node_matrix(current, :); path];
        current = tree.parent(current);
    end
    path = [path; q_goal];
    
    % ==== 关键修改：轨迹插值 ====
    N = 10;  % 每两个原始点之间插入 10 个中间点（可调整）
    interp_path = [];
    for i = 1:size(path, 1)-1
        q_start = path(i, :);
        q_end = path(i+1, :);
        t = linspace(0, 1, N+1);  % 生成 N+1 个点（包括起点）
        for j = 1:N  % 插入前 N 个点（不重复添加终点）
            q_interp = q_start + (q_end - q_start) * t(j);
            interp_path = [interp_path; q_interp];
        end
    end
    interp_path = [interp_path; path(end, :)];  % 添加最后一个原始点
    path = interp_path;  % 替换为插值后的轨迹
    
    % ==== 播放慢动作动画 ====
    % 可视化路径
    figure('Position', [1100, 300, 560, 420]);

    % 设置颜色映射
    color_map = [0.90, 0.60, 0.00;   % 橙色
             0.35, 0.70, 0.90;   % 浅蓝
             0.80, 0.40, 0.00;   % 棕红
             0.30, 0.65, 0.30];  % 深绿

    % 绘制障碍物球体
    for i = 1:size(obstacles, 1)
        [x, y, z] = sphere(50);  % 生成球面网格（分辨率50）

        % 缩放和平移球体
        x = x * obstacles(i,4) + obstacles(i,1);
        y = y * obstacles(i,4) + obstacles(i,2);
        z = z * obstacles(i,4) + obstacles(i,3);

        % 绘制带光照和透明度的球体
        surf(x, y, z, 'FaceColor', color_map(i,1:3), ...
            'EdgeColor', 'none', ...
            'FaceAlpha', 0.6, ...   % 透明度
            'FaceLighting', 'gouraud', ...
            'DiffuseStrength', 0.8, ...
            'SpecularStrength', 0.3);
        hold on;
    end

    % 添加光照和背景效果
    light('Position', [1 1 1], 'Style', 'infinite');
    lighting gouraud;       % 光滑光照模式
    material dull;          % 材质反射属性
    axis equal vis3d;       % 保持三维坐标系比例
    view(87, 30);           % 调整视角
    % plot3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 'ro', 'MarkerSize', 10);
    robot.plot(path, 'trail', 'r-', 'fps', 5);
    hold on;
%     figure(2);
%     plot3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 'ro', 'MarkerSize', 10);
%     hold on;
%     robot.plot(path, 'trail', 'r-', 'fps', 5);  % 调整 fps 控制速度（值越小越慢）
else
    disp('未找到可行路径!');
end

%% 计算总耗时
elapsed_time = toc;  % 停止计时器
fprintf('算法总运行时间: %.2f 秒\n', elapsed_time);
% run('human_draw.m');

%% 辅助函数（保持不变）
function collision = checkPathCollision(robot, q1, q2, obstacles, step)
    n_steps = max(ceil(norm(q2 - q1)/step), 3);
    t = linspace(0, 1, n_steps);
    collision = false;
    for i = 1:n_steps
        q = q1 + (q2 - q1) * t(i);
        if checkSingleConfigCollision(robot, q, obstacles)
            collision = true; return;
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
                collision = true; return;
            end
        end
    end
end

function positions = getJointPositions(robot, q)
    n = robot.n;
    positions = cell(1, n+1);
    T = robot.base;
    positions{1} = T.t(1:3)';
    for i = 1:n
        T = T * robot.links(i).A(q(i));
        positions{i+1} = T.t(1:3)';
    end
end