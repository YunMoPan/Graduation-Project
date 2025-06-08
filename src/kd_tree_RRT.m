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

%% 参数设置
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

%% RRT初始化
tree.nodes = q_start;
tree.parent = 0;
goal_reached = false;

% KDTree初始化
kdtree = KDTreeSearcher(tree.nodes);

%% 主循环
figure('Position', [440, 300, 560, 420]);
robot.plot(q_start); hold on;
% plot3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 'ro', 'MarkerSize', 10);
% 设置颜色映射
color_map = [linspace(0.0, 0.0, 4)', ...    % R: 深蓝 → 蓝 → 青 → 黄
             linspace(0.2, 0.8, 4)', ...    % G: 低 → 中 → 高 → 最高
             linspace(0.8, 0.2, 4)'];       % B: 高 → 中 → 低 → 最低


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
view(-30, 30);           % 调整视角
h_viz = plot3(nan, nan, nan, 'b.', 'MarkerSize', 5); % 预创建可视化对象
drawnow;

for iter = 1:max_iter
    % 随机采样
    if rand < prob_goal
        q_rand = q_goal;
    else
        q_rand = q_min + (q_max - q_min).*rand(1,7);
    end
    
    % KDTree最近邻搜索
    [idx_near, ~] = knnsearch(kdtree, q_rand, 'K', 1);
    q_near = tree.nodes(idx_near,:);
    
    % 扩展新节点
    direction = q_rand - q_near;
    dist = norm(direction);
    if dist > 0
        q_new = q_near + (direction / dist) * min(delta, dist);
    else
        q_new = q_near;
    end
    
    % 关节限制检查
    if any(q_new < q_min) || any(q_new > q_max)
        continue;
    end
    
    % 碰撞检测
    if ~checkPathCollision(robot, q_near, q_new, obstacles, 0.05)
        % 更新数据结构
        tree.nodes = [tree.nodes; q_new];
        tree.parent = [tree.parent; idx_near];
        
        % 增量更新KDTree
        kdtree = KDTreeSearcher(tree.nodes);
        
        % 批量更新可视化
        T_new = robot.fkine(q_new);
        set(h_viz, 'XData', [get(h_viz,'XData') T_new.t(1)],...
                  'YData', [get(h_viz,'YData') T_new.t(2)],...
                  'ZData', [get(h_viz,'ZData') T_new.t(3)]);
        if mod(iter, 50) == 0
            drawnow limitrate;
        end
        
        % 目标检查
        if norm(q_new - q_goal) < goal_tolerance
            disp('目标已到达!');
            goal_reached = true;
            break;
        end
    end
end

%% 路径回溯
if goal_reached
    % 使用KDTree找到最近目标节点
    [goal_idx, ~] = knnsearch(kdtree, q_goal);
    
    % 回溯路径
    path = [];
    current = goal_idx;
    while current ~= 0
        path = [tree.nodes(current,:); path];
        current = tree.parent(current);
    end
    
    % 可视化路径
    figure('Position', [1100, 300, 560, 420]);

    % 设置颜色映射
    color_map = [linspace(0.0, 0.0, 4)', ...    % R: 深蓝 → 蓝 → 青 → 黄
             linspace(0.2, 0.8, 4)', ...    % G: 低 → 中 → 高 → 最高
             linspace(0.8, 0.2, 4)'];       % B: 高 → 中 → 低 → 最低

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
    view(-30, 30);           % 调整视角
    % plot3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 'ro', 'MarkerSize', 10);
    robot.plot(path, 'trail', 'r-', 'fps', 30);
    hold on;
else
    disp('未找到可行路径!');
end

%% 计算总耗时
elapsed_time = toc;  % 停止计时器
fprintf('算法总运行时间: %.2f 秒\n', elapsed_time);
% run('human_draw.m');

%% 辅助函数
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