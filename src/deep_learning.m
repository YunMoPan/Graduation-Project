%% 7DOF机械臂避障轨迹规划
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
if ~exist('obstacle_num', 'var') 
  obstacle_num = 4;
end

Obstacles = [-0.1 -0.2 -0.1 0.05;
             0 -0.2 -0.4 0.05;
            0.1 -0.3 0.1 0.05;
            0.1 -0.4 -0.1 0.05]; 

% 障碍物定义 [x,y,z,radius]
obstacles = Obstacles(1:obstacle_num, :); 

% 起始/目标位形
q_start = deg2rad([-31.20, -35.63, -28.07, 70.87, -24.63, -68.56, 52.63]);
q_goal  = deg2rad([-8, 3.9, 93.9, -101.7, -11.7, 3.9, -17.8]);

%% 3. 改进的深度学习碰撞检测模型
collision_model = create_improved_collision_model(robot, obstacles, q_min, q_max);

%% 4. 生成初始轨迹
s_max = 5;
run('P2P_free_motions.m');
num_points = 120;  % 轨迹点数
Q_init = q_traj;

%% 5. 优化方法选择
fprintf('===== 使用梯度优化轨迹 =====\n');

% 梯度优化参数
opt_params = struct();
opt_params.safe_dist = 0.05;      % 安全距离阈值
opt_params.lambda_smooth = 0.3;   % 平滑性权重
opt_params.max_iter = 15;         % 迭代次数
opt_params.learning_rate = 0.02;  % 学习率
opt_params.min_improve = 1e-4;    % 最小改进阈值

% 执行梯度优化
Q_optimized = enhanced_trajectory_optimization(robot, Q_init, obstacles, ...
                    q_min, q_max, collision_model, opt_params);

%% 6. 可视化结果
visualize_results(robot, Q_optimized, obstacles, q_start, q_goal);

%% 7. 性能评估
fprintf('总耗时: %.2f 秒\n', toc);
% path = Q_optimized;
% run('human_draw.m');

%% 轨迹相似度详细评估
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

function model = create_improved_collision_model(robot, obstacles, q_min, q_max)
    fprintf('创建改进的碰撞检测模型...\n');
    
    % 增加样本量并确保均衡采样
    numSamples = 3000;
    X = zeros(numSamples, 7);
    Y = zeros(numSamples, 1);
    
    % 生成安全样本和危险样本
    safe_samples = round(numSamples * 0.6);
    unsafe_samples = numSamples - safe_samples;
    
    % 生成安全配置样本
    for i = 1:safe_samples
        while true
            q = q_min + rand(1,7).*(q_max - q_min);
            [min_dist, is_collision] = check_collision(robot, q, obstacles);
            if min_dist > 0.08  % 安全样本
                X(i,:) = q;
                Y(i) = min_dist;
                break;
            end
        end
    end
    
    % 生成危险配置样本
    for i = safe_samples+1:numSamples
        while true
            q = q_min + rand(1,7).*(q_max - q_min);
            [min_dist, is_collision] = check_collision(robot, q, obstacles);
            if min_dist < 0.06  % 危险样本
                X(i,:) = q;
                Y(i) = min_dist;
                break;
            end
        end
    end
    
    % 修正的网络结构 - 添加回归层作为输出层
    layers = [
        featureInputLayer(7, 'Name', 'input')
        fullyConnectedLayer(128, 'Name', 'fc1')
        batchNormalizationLayer('Name','bn1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(64, 'Name', 'fc2')
        batchNormalizationLayer('Name','bn2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(32, 'Name', 'fc3')
        reluLayer('Name', 'relu3')
        fullyConnectedLayer(1, 'Name', 'output')
        regressionLayer('Name', 'regression')];  % 添加回归层作为输出层
    
    % 训练选项
    options = trainingOptions('adam', ...
        'MaxEpochs', 50, ...
        'MiniBatchSize', 128, ...
        'InitialLearnRate', 0.001, ...
        'LearnRateSchedule', 'piecewise', ...
        'LearnRateDropFactor', 0.5, ...
        'LearnRateDropPeriod', 15, ...
        'Shuffle', 'every-epoch', ...
        'Plots', 'training-progress', ...
        'Verbose', true);
    
    % 训练网络
    model = trainNetwork(X, Y, layers, options);
    
    % 保存模型测试性能
    test_idx = randperm(numSamples, floor(numSamples*0.2));
    X_test = X(test_idx,:);
    Y_test = Y(test_idx);
    Y_pred = predict(model, X_test);
    
    mse_test = mean((Y_test - Y_pred).^2);
    fprintf('测试集MSE: %.4f\n', mse_test);
end

%% 增强的轨迹优化函数 (梯度方法)
function Q_opt = enhanced_trajectory_optimization(robot, Q_init, obstacles, ...
                          q_min, q_max, collision_model, params)
    fprintf('执行增强轨迹优化...\n');
    
    Q_opt = Q_init;
    n = size(Q_opt, 1);
    prev_cost = inf;
    
    % 计算初始轨迹安全成本
    init_safety_cost = 0;
    for i = 1:n
        q = Q_opt(i,:);
        safety = predict(collision_model, q);
        init_safety_cost = init_safety_cost + max(0, params.safe_dist - safety);
    end
    init_safety_cost = init_safety_cost / n;
    
    % 批量梯度优化
    for iter = 1:params.max_iter
        total_improvement = 0;
        current_lr = params.learning_rate * (1 - iter/params.max_iter)^0.5;
        
        % 计算全轨迹梯度
        grad_total = zeros(size(Q_opt));
        for i = 2:n-1
            [~, grad] = point_cost(Q_opt, i, collision_model, params);
            grad_total(i,:) = grad;
        end
        
        % 应用批量更新
        Q_new = Q_opt - current_lr * grad_total;
        
        % 应用关节限制
        Q_new = max(min(Q_new, q_max), q_min);
        
        % 计算新成本
        new_cost = 0;
        for i = 1:n
            [cost_i, ~] = point_cost(Q_new, i, collision_model, params);
            new_cost = new_cost + cost_i;
        end
        new_cost = new_cost / n;
        
        % 计算旧成本
        old_cost = 0;
        for i = 1:n
            [cost_i, ~] = point_cost(Q_opt, i, collision_model, params);
            old_cost = old_cost + cost_i;
        end
        old_cost = old_cost / n;
        
        % 接受改进
        if new_cost < old_cost
            Q_opt = Q_new;
            total_improvement = old_cost - new_cost;
        end
        
        % 应用自适应平滑
        smooth_window = max(3, min(9, round(iter/2)));
        Q_opt = smoothdata(Q_opt, 'gaussian', smooth_window);
        
        % 计算整个轨迹的安全成本
        safety_cost = 0;
        for i = 1:n
            q = Q_opt(i,:);
            safety = predict(collision_model, q);
            safety_cost = safety_cost + max(0, params.safe_dist - safety);
        end
        safety_cost = safety_cost / n;
        
        fprintf('迭代 %d/%d: 安全成本=%.4f, 改进=%.4f\n', ...
                iter, params.max_iter, safety_cost, total_improvement);
        
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
        q = Q_opt(i,:);
        safety = predict(collision_model, q);
        final_safety = final_safety + safety;
        if safety < min_safety
            min_safety = safety;
        end
    end
    avg_safety = final_safety / size(Q_opt,1);
    
    fprintf('优化完成: 初始安全成本=%.4f, 最终平均安全距离=%.4f, 最小安全距离=%.4f\n', ...
            init_safety_cost, avg_safety, min_safety);
end

%% 计算轨迹点成本
function [cost, grad] = point_cost(Q, idx, model, params)
    q = Q(idx,:);
    
    % 预测安全距离
    safety = predict(model, q);
    
    % 计算安全成本（线性惩罚）
    safety_cost = max(0, params.safe_dist - safety);
    
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
    
    % 数值梯度计算（更稳定）
    epsilon = 1e-6;
    grad = zeros(1,7);
    for j = 1:7
        q_plus = q;
        q_plus(j) = q_plus(j) + epsilon;
        cost_plus = point_cost_single(q_plus, model, params);
        
        q_minus = q;
        q_minus(j) = q_minus(j) - epsilon;
        cost_minus = point_cost_single(q_minus, model, params);
        
        grad(j) = (cost_plus - cost_minus) / (2*epsilon);
    end
    
    % 添加平滑梯度
    grad = grad + params.lambda_smooth * grad_smooth;
    
    % 嵌套单点成本函数
    function cost = point_cost_single(q, model, params)
        safety = predict(model, q);
        safety_cost = max(0, params.safe_dist - safety);
        cost = safety_cost;
    end
end

%% 可视化结果
function visualize_results(robot, Q, obstacles, q_start, q_goal)
    % 碰撞检查
    min_safety = Inf;
    for i = 1:size(Q,1)
        [min_dist, ~] = check_collision(robot, Q(i,:), obstacles);
        if min_dist < min_safety
            min_safety = min_dist;
        end
    end
    
    % 创建可视化
    figure('Name','优化轨迹','Position',[100 100 560 420]);
    
    % 绘制障碍物
    [X,Y,Z] = sphere(8);  % 进一步简化球体网格
    for obs_idx = 1:size(obstacles,1)
        obs = obstacles(obs_idx,:);
        surf(X*obs(4)+obs(1), Y*obs(4)+obs(2), Z*obs(4)+obs(3),...
            'FaceColor',[0.9 0.2 0.2],'FaceAlpha',0.3,'EdgeColor','none');
        hold on;
    end
    
    % 绘制轨迹
    robot.plot(Q, 'trail','b-','fps',15,'view',[-70 30], 'nobase', 'noname', 'nowrist', 'noshadow');
    title(sprintf('优化轨迹 (最小安全距离: %.4f m)', min_safety));
    
    % 添加起始和目标点标记
    T_start = robot.fkine(q_start);
    T_goal = robot.fkine(q_goal);
    plot3(T_start.t(1), T_start.t(2), T_start.t(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(T_goal.t(1), T_goal.t(2), T_goal.t(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    legend('障碍物', '轨迹', '起点', '终点');
    
    % 绘制关节角度变化
    figure;
    plot(rad2deg(Q));
    title('关节角度变化');
    xlabel('时间步');
    ylabel('角度 (度)');
    grid on;
end

%% 碰撞检测函数
function [min_dist, is_collision] = check_collision(robot, q, obstacles)
    min_dist = Inf;
    is_collision = false;
    prev_pos = [0 0 0];
    
    % 简化：只检查关键连杆
    key_links = [3, 5, 7];  % 选择几个关键连杆
    
    for link_idx = key_links
        T = robot.A(link_idx, q).T;
        curr_pos = T(1:3,4)';
        
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
        prev_pos = curr_pos;
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