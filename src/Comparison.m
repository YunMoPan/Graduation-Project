%% 7DOF机械臂轨迹优化方法对比实验
clear; clc; close all;

%% 1. 实验参数设置
num_trials = 10;                % 重复试验次数
complexity_levels = [1, 2, 3];  % 场景复杂度：1=简单，2=中等，3=复杂
results = struct();             % 存储实验结果
s_max = 5;
run('P2P_free_motions.m');

%% 2. 初始化机械臂模型
L(1) = Link('d', -0.04, 'a', 0,     'alpha', pi/2, 'offset', 0, 'standard');
L(2) = Link('d', 0,     'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'standard');
L(3) = Link('d', 0,     'a', -0.3,  'alpha', 0,    'offset', 0, 'standard');
L(4) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', -pi/2, 'standard');
L(5) = Link('d', 0,     'a', 0.25,  'alpha', -pi/2, 'offset', 0, 'standard');
L(6) = Link('d', 0,     'a', 0.04,  'alpha', pi/2, 'offset', 0, 'standard');
L(7) = Link('d', 0,     'a', 0,     'alpha', 0,    'offset', 0, 'standard');
robot = SerialLink(L, 'name', '7DOF-Arm');

%% 3. 定义不同复杂度的测试场景
scenarios = generate_test_scenarios(complexity_levels);

%% 4. 执行对比实验（改进版）
for s = 1:length(scenarios)
    s = 3;  % 固定场景为3，可能需要移除这行以测试所有场景
    scenario = scenarios(s);
    fprintf('\n===== 测试场景 %d: %s =====\n', s, scenario.name);
    
    optionsA = struct();
    optionsA.safe_dist = 0.05;
    optionsA.lambda_smooth = 0.3;
    optionsA.max_iter = 15;
    optionsA.learning_rate = 0.01; % 学习率
    optionsA.min_improve = 1e-6;   % 减小收敛阈值
    optionsA.early_stopping = true; % 启用早停机制
    optionsA.patience = 2;         % 早停等待轮数

    % 训练碰撞模型
    tic
    collision_model = create_improved_collision_model(robot, scenario.obstacles, -pi*ones(1,7), pi*ones(1,7));
    time_C = toc;
    
    % 初始化结果存储
    methodA_times = zeros(num_trials, 1);
    methodA_safety = zeros(num_trials, 1);
    methodA_smoothness = zeros(num_trials, 1);
    
    methodB_times = zeros(num_trials, 1);
    methodB_safety = zeros(num_trials, 1);
    methodB_smoothness = zeros(num_trials, 1);
    
    % 执行多次试验
    for trial = 1:num_trials
        fprintf('\n- 试验 %d/%d -\n', trial, num_trials);
        
        % 生成随机初始轨迹（确保可比性）
        q_start = scenario.q_start;
        q_goal = scenario.q_goal;
        Q_init = q_traj;
        
        % 方法A：深度学习优化（代码1）
        fprintf('运行方法A（深度学习优化）...\n');
        
        % 执行优化
        tic;
        Q_optimized_A = enhanced_trajectory_optimization(robot, Q_init, scenario.obstacles, ...
            -pi*ones(1,7), pi*ones(1,7), collision_model, optionsA);
        time_A = toc;
        
        % 方法B：传统优化（代码2）
        fprintf('运行方法B（传统优化）...\n');
        tic;
        Q_optimized_B = optimize_trajectory(robot, Q_init, scenario.obstacles, -pi*ones(1,7), pi*ones(1,7));
        time_B = toc;
        
        % 评估并存储结果
        eval_A = evaluate_trajectory(robot, Q_init, Q_optimized_A, scenario.obstacles, time_A);
        eval_B = evaluate_trajectory(robot, Q_init, Q_optimized_B, scenario.obstacles, time_B);
        
        % 存储单次试验结果
        methodA_times(trial) = eval_A.time;
        methodA_safety(trial) = eval_A.final_avg_safety;
        methodA_smoothness(trial) = eval_A.smoothness;
        
        methodB_times(trial) = eval_B.time;
        methodB_safety(trial) = eval_B.final_avg_safety;
        methodB_smoothness(trial) = eval_B.smoothness;
        
        % 打印单次试验结果
        fprintf('方法A: 耗时=%.2fs, 平均安全距离=%.4fm, 平滑度=%.4f\n', ...
            eval_A.time, eval_A.final_avg_safety, eval_A.smoothness);
        fprintf('方法B: 耗时=%.2fs, 平均安全距离=%.4fm, 平滑度=%.4f\n', ...
            eval_B.time, eval_B.final_avg_safety, eval_B.smoothness);
    end
    
    % 汇总分析所有试验结果
    fprintf('\n===== 场景 %d 所有试验汇总分析 =====\n', s);
    
    % 计算统计指标
    statsA = struct(...
        'mean_time', mean(methodA_times),...
        'std_time', std(methodA_times),...
        'mean_safety', mean(methodA_safety),...
        'std_safety', std(methodA_safety),...
        'mean_smoothness', mean(methodA_smoothness),...
        'std_smoothness', std(methodA_smoothness),...
        'success_rate', sum(methodA_safety > 0.05) / num_trials);
    
    statsB = struct(...
        'mean_time', mean(methodB_times),...
        'std_time', std(methodB_times),...
        'mean_safety', mean(methodB_safety),...
        'std_safety', std(methodB_safety),...
        'mean_smoothness', mean(methodB_smoothness),...
        'std_smoothness', std(methodB_smoothness),...
        'success_rate', sum(methodB_safety > 0.05) / num_trials);
    
    % 打印汇总结果
    fprintf('\n方法A 统计结果 (共%d次试验):\n', num_trials);
    fprintf('平均耗时: %.2f±%.2f s\n', statsA.mean_time, statsA.std_time);
    fprintf('平均安全距离: %.4f±%.4f m\n', statsA.mean_safety, statsA.std_safety);
    fprintf('平均平滑度: %.4f±%.4f\n', statsA.mean_smoothness, statsA.std_smoothness);
    fprintf('成功率 (安全距离>0.05m): %.2f%%\n', statsA.success_rate * 100);
    
    fprintf('\n方法B 统计结果 (共%d次试验):\n', num_trials);
    fprintf('平均耗时: %.2f±%.2f s\n', statsB.mean_time, statsB.std_time);
    fprintf('平均安全距离: %.4f±%.4f m\n', statsB.mean_safety, statsB.std_safety);
    fprintf('平均平滑度: %.4f±%.4f\n', statsB.mean_smoothness, statsB.std_smoothness);
    fprintf('成功率 (安全距离>0.05m): %.2f%%\n', statsB.success_rate * 100);
    
    % 比较两种方法
    fprintf('\n===== 方法对比 =====\n');
    fprintf('方法A vs 方法B:\n');
end

%% 4. 执行对比实验
for s = 1:length(scenarios)
    s = 3;
    scenario = scenarios(s);
    fprintf('\n===== 测试场景 %d: %s =====\n', s, scenario.name);
    
    optionsA = struct();
    optionsA.safe_dist = 0.05;
    optionsA.lambda_smooth = 0.3;
    optionsA.max_iter = 15;
    optionsA.learning_rate = 0.01; % 学习率
    optionsA.min_improve = 1e-6;   % 减小收敛阈值
    optionsA.early_stopping = true; % 启用早停机制
    optionsA.patience = 2;         % 早停等待轮数

    % 训练碰撞模型
    tic
    collision_model = create_improved_collision_model(robot, scenario.obstacles, -pi*ones(1,7), pi*ones(1,7));
    time_C = toc;
    
    for trial = 1:num_trials
        fprintf('\n- 试验 %d/%d -\n', trial, num_trials);
        
        % 生成随机初始轨迹（确保可比性）
        q_start = scenario.q_start;
        q_goal = scenario.q_goal;
        Q_init = q_traj;
        
        % 方法A：深度学习优化（代码1）
        fprintf('运行方法A（深度学习优化）...\n');
        
        % 执行优化
        tic;
        Q_optimized_A = enhanced_trajectory_optimization(robot, Q_init, scenario.obstacles, ...
            -pi*ones(1,7), pi*ones(1,7), collision_model, optionsA);
        time_A = toc;
        
        % 方法B：传统优化（代码2）
        fprintf('运行方法B（传统优化）...\n');
        tic;
        Q_optimized_B = optimize_trajectory(robot, Q_init, scenario.obstacles, -pi*ones(1,7), pi*ones(1,7));
        time_B = toc;
        
        % 评估优化结果
        results(s).trials(trial).methodA = evaluate_trajectory(robot, Q_init, Q_optimized_A, scenario.obstacles, time_A);
        results(s).trials(trial).methodB = evaluate_trajectory(robot, Q_init, Q_optimized_B, scenario.obstacles, time_B);
        
        % 打印方法A和方法B的结果（确保字段存在）
        fprintf('方法A: 耗时=%.2fs, 平均安全距离=%.4fm, 平滑度=%.4f\n', ...
            results(s).trials(trial).methodA.time, ...
            results(s).trials(trial).methodA.final_avg_safety, ... 
            results(s).trials(trial).methodA.smoothness);
        
        fprintf('方法B: 耗时=%.2fs, 平均安全距离=%.4fm, 平滑度=%.4f\n', ...
            results(s).trials(trial).methodB.time, ...
            results(s).trials(trial).methodB.final_avg_safety, ... 
            results(s).trials(trial).methodB.smoothness);
        disp(time_C);
    end
end

%% 5. 结果分析与可视化
% analyze_results(results, scenarios);

%% 辅助函数：生成测试场景
function scenarios = generate_test_scenarios(complexity_levels)
    scenarios = struct();
    idx = 1;
    
    % 简单场景：4个障碍物（原始场景）
    obstacles_simple = [-0.1 -0.2 -0.1 0.05;
                         0 -0.2 -0.4 0.05;
                        0.1 -0.3 0.1 0.05;
                        0.1 -0.4 -0.1 0.05];
    q_start = deg2rad([-31.20, -35.63, -28.07, 70.87, -24.63, -68.56, 52.63]);
    q_goal  = deg2rad([-8, 3.9, 93.9, -101.7, -11.7, 3.9, -17.8]);
    
    scenarios(idx).name = '简单场景';
    scenarios(idx).obstacles = obstacles_simple;
    scenarios(idx).q_start = q_start;
    scenarios(idx).q_goal = q_goal;
    idx = idx + 1;
    
    % 中等复杂度场景：8个障碍物（增加密度）
    obstacles_medium = [-0.1 -0.2 -0.1 0.05;
                         0 -0.2 -0.4 0.05;
                        0.1 -0.3 0.1 0.05;
                        0.1 -0.4 -0.1 0.05;
                        -0.2 -0.3 -0.2 0.06;
                         0.2 -0.3 -0.3 0.06;
                         0 -0.4 0 0.06;
                        -0.1 -0.5 -0.2 0.06];
    
    scenarios(idx).name = '中等复杂度场景';
    scenarios(idx).obstacles = obstacles_medium;
    scenarios(idx).q_start = q_start;
    scenarios(idx).q_goal = q_goal;
    idx = idx + 1;
    
    % 复杂场景：包含动态障碍物
    if any(complexity_levels >= 3)
        % 静态障碍物
        obstacles_static = [-0.1 -0.2 -0.1 0.05;
                             0 -0.2 -0.4 0.05;
                            0.1 -0.3 0.1 0.05;
                            0.1 -0.4 -0.1 0.05;
                            -0.2 -0.3 -0.2 0.06];
        
        % 动态障碍物参数（位置随时间变化）
        obstacles_dynamic = struct();
        obstacles_dynamic(1).center = [0.2 -0.3 -0.3];
        obstacles_dynamic(1).radius = 0.06;
        obstacles_dynamic(1).amplitude = [0.1 0 0.1];
        obstacles_dynamic(1).frequency = [0.1 0 0.1];
        
        obstacles_dynamic(2).center = [0 -0.4 0];
        obstacles_dynamic(2).radius = 0.06;
        obstacles_dynamic(2).amplitude = [0 0.1 0.1];
        obstacles_dynamic(2).frequency = [0 0.1 0.1];
        
        scenarios(idx).name = '复杂场景（含动态障碍物）';
        scenarios(idx).obstacles = obstacles_static;
        scenarios(idx).dynamic_obstacles = obstacles_dynamic;
        scenarios(idx).q_start = q_start;
        scenarios(idx).q_goal = q_goal;
        idx = idx + 1;
    end
end

%% 辅助函数：评估轨迹质量
function metrics = evaluate_trajectory(robot, Q_init, Q_optimized, obstacles, time)
    % 计算初始轨迹安全指标（与原代码一致）
    init_min_safety = inf;
    init_avg_safety = 0;
    for i = 1:size(Q_init,1)
        [min_dist, ~] = check_collision(robot, Q_init(i,:), obstacles);
        init_avg_safety = init_avg_safety + min_dist;
        if min_dist < init_min_safety
            init_min_safety = min_dist;
        end
    end
    init_avg_safety = init_avg_safety / size(Q_init,1);

    % 计算优化轨迹安全指标（补充 avg_safety）
    final_min_safety = inf;
    final_avg_safety = 0;  % 添加平均安全距离
    collision_count = 0;
    for i = 1:size(Q_optimized,1)
        [min_dist, is_collision] = check_collision(robot, Q_optimized(i,:), obstacles);
        final_avg_safety = final_avg_safety + min_dist;  % 累加距离
        if min_dist < final_min_safety
            final_min_safety = min_dist;
        end
        if is_collision
            collision_count = collision_count + 1;
        end
    end
    final_avg_safety = final_avg_safety / size(Q_optimized,1);  % 计算平均值

    % 其他指标（平滑度、偏差等，与原代码一致）
    smoothness = 0;
    for i = 2:size(Q_optimized,1)
        smoothness = smoothness + norm(Q_optimized(i,:) - Q_optimized(i-1,:));
    end
    smoothness = smoothness / (size(Q_optimized,1) - 1);

    deviation = 0;
    for i = 1:size(Q_optimized,1)
        deviation = deviation + norm(Q_optimized(i,:) - Q_init(i,:));
    end
    deviation = deviation / size(Q_optimized,1);

    % 统一结构体字段（两种方法保持一致）
    metrics.time = time;
    metrics.init_min_safety = init_min_safety;
    metrics.init_avg_safety = init_avg_safety;
    metrics.final_min_safety = final_min_safety;
    metrics.final_avg_safety = final_avg_safety;  % 新增字段
    metrics.safety_improvement = (final_avg_safety - init_avg_safety) / init_avg_safety * 100;
    metrics.collision_count = collision_count;
    metrics.smoothness = smoothness;
    metrics.deviation = deviation;
end

%% 辅助函数：分析和可视化结果
function analyze_results(results, scenarios)
    % 创建结果汇总表
    method_names = {'深度学习优化', '传统优化'};
    metrics = {'安全距离提升(%)', '平均安全距离(m)', '最小安全距离(m)', '碰撞次数', '平滑度', '运行时间(s)'};
    
    % 为每个场景绘制对比图
    for s = 1:length(scenarios)
        figure('Name', sprintf('场景 %d: %s 对比结果', s, scenarios(s).name));
        
        % 提取数据
        data_A = zeros(length(results(s).trials), length(metrics));
        data_B = zeros(length(results(s).trials), length(metrics));
        
        for trial = 1:length(results(s).trials)
            data_A(trial,:) = [results(s).trials(trial).methodA.safety_improvement, ...
                              results(s).trials(trial).methodA.final_avg_safety, ...
                              results(s).trials(trial).methodA.final_min_safety, ...
                              results(s).trials(trial).methodA.collision_count, ...
                              results(s).trials(trial).methodA.smoothness, ...
                              results(s).trials(trial).methodA.time];
            
            data_B(trial,:) = [results(s).trials(trial).methodB.safety_improvement, ...
                              results(s).trials(trial).methodB.final_avg_safety, ...
                              results(s).trials(trial).methodB.final_min_safety, ...
                              results(s).trials(trial).methodB.collision_count, ...
                              results(s).trials(trial).methodB.smoothness, ...
                              results(s).trials(trial).methodB.time];
        end
        
        % 计算平均值和标准差
        mean_A = mean(data_A);
        std_A = std(data_A);
        mean_B = mean(data_B);
        std_B = std(data_B);
        
        % 绘制柱状图对比
        x = 1:length(metrics);
        width = 0.35;
        
        figure;
        bar(x-width/2, mean_A, width, 'b', 'FaceAlpha', 0.7);
        hold on;
        bar(x+width/2, mean_B, width, 'r', 'FaceAlpha', 0.7);
        
        % 添加误差线
        errorbar(x-width/2, mean_A, std_A, 'b.', 'LineWidth', 1);
        errorbar(x+width/2, mean_B, std_B, 'r.', 'LineWidth', 1);
        
        % 设置图表属性
        title(sprintf('场景 %d: %s 优化方法对比', s, scenarios(s).name));
        set(gca, 'XTick', x, 'XTickLabel', metrics);
        ylabel('指标值');
        legend(method_names);
        grid on;
        
        % 输出数值结果
        fprintf('\n===== 场景 %d: %s 结果汇总 =====\n', s, scenarios(s).name);
        fprintf('指标\t\t\t%s\t\t%s\t\t差异(%%)\n', method_names{1}, method_names{2});
        for m = 1:length(metrics)
            diff_percent = (mean_A(m) - mean_B(m)) / mean_B(m) * 100;
            fprintf('%s\t\t%.4f±%.4f\t%.4f±%.4f\t%.2f%%\n', ...
                metrics{m}, mean_A(m), std_A(m), mean_B(m), std_B(m), diff_percent);
        end
    end
    
    % 绘制总体性能对比雷达图
    figure('Name', '总体性能对比雷达图');
    
    % 计算各场景的平均性能
    overall_A = zeros(length(metrics), length(scenarios));
    overall_B = zeros(length(metrics), length(scenarios));
    
    for s = 1:length(scenarios)
        for trial = 1:length(results(s).trials)
            overall_A(:,s) = overall_A(:,s) + [results(s).trials(trial).methodA.safety_improvement;
                                               results(s).trials(trial).methodA.final_avg_safety;
                                               results(s).trials(trial).methodA.final_min_safety;
                                               100 - results(s).trials(trial).methodA.collision_count;  % 成功率
                                               results(s).trials(trial).methodA.smoothness;
                                               1/results(s).trials(trial).methodA.time];  % 速度倒数
            
            overall_B(:,s) = overall_B(:,s) + [results(s).trials(trial).methodB.safety_improvement;
                                               results(s).trials(trial).methodB.final_avg_safety;
                                               results(s).trials(trial).methodB.final_min_safety;
                                               100 - results(s).trials(trial).methodB.collision_count;  % 成功率
                                               results(s).trials(trial).methodB.smoothness;
                                               1/results(s).trials(trial).methodB.time];  % 速度倒数
        end
        
        % 取平均值
        overall_A(:,s) = overall_A(:,s) / length(results(s).trials);
        overall_B(:,s) = overall_B(:,s) / length(results(s).trials);
    end
    
    % 归一化处理
    for m = 1:length(metrics)
        max_val = max([overall_A(m,:), overall_B(m,:)]);
        min_val = min([overall_A(m,:), overall_B(m,:)]);
        overall_A(m,:) = (overall_A(m,:) - min_val) / (max_val - min_val);
        overall_B(m,:) = (overall_B(m,:) - min_val) / (max_val - min_val);
    end
    
    % 绘制雷达图
    for s = 1:length(scenarios)
        subplot(1, length(scenarios), s);
        
        % 数据准备
        angles = linspace(0, 2*pi, length(metrics)+1);
        data_A_plot = [overall_A(:,s); overall_A(1,s)];
        data_B_plot = [overall_B(:,s); overall_B(1,s)];
        
        % 绘制多边形
        polarplot(angles, data_A_plot, 'b-', 'LineWidth', 2);
        hold on;
        polarplot(angles, data_B_plot, 'r-', 'LineWidth', 2);
        
        % 填充区域
        fill(angles, data_A_plot, 'b', 'FaceAlpha', 0.2);
        fill(angles, data_B_plot, 'r', 'FaceAlpha', 0.2);
        
        % 设置标签
        set(gca, 'RTick', [0.2, 0.4, 0.6, 0.8, 1.0]);
        set(gca, 'ThetaTick', angles(1:end-1)*180/pi);
        set(gca, 'ThetaTickLabel', metrics);
        title(scenarios(s).name);
        legend(method_names);
    end
end

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