%% 计时开始
tic;  % 启动计时器

%% 机械臂定义
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'standard');
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0, 'standard');
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2, 'standard');
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0, 'standard');
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0, 'standard');
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0, 'standard');
robot = SerialLink(L, 'name', '7DOF-Arm');

%% 参数设置（新增树切换阈值）
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
max_iter = 3000;
delta = 0.2;
goal_tolerance = 0.3;
prob_goal = 0.1;
connect_threshold = 0.5;  % 连接尝试阈值

%% RRT-Connect初始化（双树结构）
current_tree = 2;
% 起点树（Tree A）初始化
treeA.nodes = q_start;
treeA.parent = 0;
treeA.end_effector_pos = robot.fkine(q_start).t(1:3)';  % 新增：存储末端坐标
treeA.kdtree = KDTreeSearcher(treeA.nodes);

% 目标点树（Tree B）初始化
treeB.nodes = q_goal;
treeB.parent = 0;
treeB.end_effector_pos = robot.fkine(q_goal).t(1:3)';  % 新增：存储末端坐标
treeB.kdtree = KDTreeSearcher(treeB.nodes);

%% 可视化初始化
figure('Position', [440, 300, 560, 420]);
robot.plot(q_start); hold on;

% 颜色映射设置
color_map = [
    0.8, 0.2, 0.1;  % 红色变体
    0.9, 0.4, 0.2;  % 橙色变体
    1.0, 0.8, 0.3;  % 黄色变体
    0.7, 0.9, 0.5   % 绿色变体
];

% 绘制障碍物球体（保持原样）
for i = 1:size(obstacles, 1)
    [x, y, z] = sphere(50);  
    x = x * obstacles(i,4) + obstacles(i,1);
    y = y * obstacles(i,4) + obstacles(i,2);
    z = z * obstacles(i,4) + obstacles(i,3);
    surf(x, y, z, 'FaceColor', color_map(i,1:3), ...
        'EdgeColor', 'none', 'FaceAlpha', 0.6);
end

% 光照与视角设置
light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud; material dull; axis equal vis3d; view(-30, 30);

% 双树可视化句柄
h_treeA = plot3(nan, nan, nan, 'b.', 'MarkerSize', 5); % 树A：蓝色
h_treeB = plot3(nan, nan, nan, 'g.', 'MarkerSize', 5); % 树B：绿色
drawnow;

%% RRT-Connect主循环
for iter = 1:max_iter
    % 步骤1：随机采样（目标偏置）
    if rand < prob_goal
        q_rand = (current_tree == 1) * q_goal + (current_tree == 2) * q_start;
    else
        q_rand = q_min + (q_max - q_min).*rand(1,7);
    end
    
    % 步骤2：扩展当前树
    if current_tree == 1
        [new_node, is_extended] = extendTree(treeA, q_rand, delta, robot, obstacles);
        other_tree = treeB;
    else
        [new_node, is_extended] = extendTree(treeB, q_rand, delta, robot, obstacles);
        other_tree = treeA;
    end
    
    % 步骤3：尝试连接另一棵树
    if is_extended
        [is_connected, connect_path] = connectTrees(other_tree, new_node, robot, obstacles, delta);
        
        if is_connected
            goal_reached = true;
            break;
        end
        
        % 步骤4：切换当前树（动态调整策略）
        if norm(new_node - q_goal) < connect_threshold
            current_tree = 2;  % 接近目标时优先扩展Tree B
        elseif norm(new_node - q_start) < connect_threshold
            current_tree = 1;  % 接近起点时优先扩展Tree A
        else
            current_tree = 3 - current_tree;  % 常规切换
        end
    end
    
    % 可视化更新（每20次迭代刷新一次）
    updateVisualization(h_treeA, treeA);  % 更新树A的末端轨迹
    updateVisualization(h_treeB, treeB);  % 更新树B的末端轨迹
    if mod(iter, 20) == 0
        drawnow limitrate;
    end
end

%% 路径处理与可视化
if goal_reached
    % 合并双树路径
    if current_tree == 1
        path = [flipud(connect_path); getTreePath(treeA, new_node)];
    else
        path = [getTreePath(treeA, new_node); connect_path];
    end
    
    % 显示最终路径
    % 可视化路径
    figure('Position', [1100, 300, 560, 420]);

    % 设置颜色映射（从红色到橙色渐变）
    color_map = [
        0.8, 0.2, 0.1;  % 红色变体
        0.9, 0.4, 0.2;  % 橙色变体
        1.0, 0.8, 0.3;  % 黄色变体
        0.7, 0.9, 0.5   % 绿色变体
    ];

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
    view(45, 30);           % 调整视角
    % plot3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 'ro', 'MarkerSize', 10);
    robot.plot(path, 'trail', 'r-', 'fps', 30);
    hold on;
    title('RRT-Connect Generated Path');
else
    disp('未找到可行路径!');
end

%% 输出运行时间
fprintf('算法总运行时间: %.2f 秒\n', toc);
% run('human_draw.m');

%% 新增核心函数
function [new_node, is_extended] = extendTree(tree, q_rand, delta, robot, obstacles)
    [idx_near, ~] = knnsearch(tree.kdtree, q_rand);
    q_near = tree.nodes(idx_near, :);
    
    % 节点扩展
    direction = q_rand - q_near;
    dist = norm(direction);
    if dist > 0
        q_new = q_near + (direction / dist) * min(delta, dist);
    else
        is_extended = false;
        new_node = [];
        return;
    end
    
    % 碰撞检测
    if ~checkPathCollision(robot, q_near, q_new, obstacles, 0.05)
        % 新增：计算新节点末端坐标并存储
        T_new = robot.fkine(q_new);
        tree.end_effector_pos = [tree.end_effector_pos; T_new.t(1:3)'];  % 追加末端坐标
        
        % 更新树结构
        tree.nodes = [tree.nodes; q_new];
        tree.parent = [tree.parent; idx_near];
        tree.kdtree = KDTreeSearcher(tree.nodes);
        is_extended = true;
        new_node = q_new;
    else
        is_extended = false;
        new_node = [];
    end
end

function [is_connected, path] = connectTrees(tree, target_node, robot, obstacles, delta)
    [idx_near, ~] = knnsearch(tree.kdtree, target_node);
    q_near = tree.nodes(idx_near, :);
    q_target = target_node;
    
    % 逐步逼近目标节点
    while true
        direction = q_target - q_near;
        dist = norm(direction);
        if dist <= delta
            q_new = q_target;
        else
            q_new = q_near + (direction / dist) * delta;
        end
        
        % 碰撞检测
        if checkPathCollision(robot, q_near, q_new, obstacles, 0.05)
            is_connected = false;
            path = [];
            return;
        end
        
        % 添加新节点
        tree.nodes = [tree.nodes; q_new];
        tree.parent = [tree.parent; idx_near];
        tree.kdtree = KDTreeSearcher(tree.nodes);
        idx_near = size(tree.nodes, 1);
        q_near = q_new;
        
        % 检查连接
        if norm(q_new - q_target) < 0.1
            path = getTreePath(tree, q_new);
            is_connected = true;
            return;
        end
    end
end

function path = getTreePath(tree, end_node)
    path = [];
    current = find(ismember(tree.nodes, end_node, 'rows'));
    while current ~= 0
        path = [tree.nodes(current, :); path];
        current = tree.parent(current);
    end
end

function updateVisualization(h_handle, tree)
    % 直接使用预存的末端坐标更新可视化
    set(h_handle, 'XData', tree.end_effector_pos(:,1), ...
                  'YData', tree.end_effector_pos(:,2), ...
                  'ZData', tree.end_effector_pos(:,3));
end

%% 原有碰撞检测函数（保持不变）
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
    n = robot.n;
    positions = cell(1, n+1);
    T = robot.base;
    positions{1} = T.t(1:3)';
    
    for i = 1:n
        T = T * robot.links(i).A(q(i));
        positions{i+1} = T.t(1:3)';
    end
end