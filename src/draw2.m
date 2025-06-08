%% 数据准备
run('read_data.m');
[R, M, n_dims] = size(Q); % Q维度应为 R×M×7 (样本数×时间点数×关节维度)
if ~exist('s_max', 'var') 
   s_max = 3;
end
s_max = 15;
t_fin = 10; 
t = linspace(0, t_fin, M)'; 

%% 步骤1: 计算全局平均姿态 tilde_q (强制为1×7行向量)
% 双重平均：先跨样本平均，再跨时间平均
tilde_q = squeeze(mean(mean(Q, 1), 2))'; % 添加转置操作
if ~isequal(size(tilde_q), [1, n_dims])
    error('tilde_q维度错误，应为1×7。实际维度：%s', mat2str(size(tilde_q)));
end

%% 步骤2: 计算零阶协同效应 S0(t) (M×7)
S0 = squeeze(mean(Q, 1)); % 沿样本维度平均
if ~isequal(size(S0), [M, n_dims])
    error('S0维度错误，应为M×7。实际维度：%s', mat2str(size(S0)));
end

%% 步骤3: 中心化数据 (显式维度扩展)
% 扩展全局均值到 R×M×7
tilde_q_expanded = repmat(reshape(tilde_q, 1, 1, n_dims), [R, M, 1]);

% 扩展时间相关均值到 R×M×7
S0_expanded = repmat(S0, [1, 1, R]); 
S0_expanded = permute(S0_expanded, [3, 1, 2]); % 调整为 R×M×7

% 执行双去均值化
Q_centered = Q - tilde_q_expanded - S0_expanded;

%% 步骤4: 展平数据为二维矩阵（样本×(时间×维度)）
X = zeros(R, M*n_dims);
for j = 1:R
    sample_data = squeeze(Q_centered(j, :, :)); % 提取 M×7 数据
    X(j, :) = sample_data(:)'; % 按列优先展平
end


%% 步骤5: 执行PCA（注意已提前中心化）
[coeff, score, ~, ~, explained] = pca(X, 'Centered', false);

%% 步骤6: 提取基函数 S_i(t)（M×7 形式）
S = cell(s_max, 1);
for i = 1:s_max
    % 从PCA系数中重构基函数
    Si = reshape(coeff(:, i), [M, n_dims]); % pca函数返回的coeff矩阵的列数应该是min(R-1, M*7)
    % 正则化处理（可选）
    Si = Si / norm(Si(:)); 
    S{i} = Si;
end

%% 步骤7: 重构样本轨迹并可视化
orign_data = squeeze(mean(Q, 1)); % 对所有样本求平均
alpha = mean(score(:, 1:s_max), 1); % 得到 1×s_max 的平均权重矩阵

reconstructed = alpha(1) * S{1};
%% 步骤8: 绘图
% 创建画布
figure('Position', [100 100 1400 900], 'Color', 'w');
% figure('Position', [100 100 1920 1080], 'Color', 'w');
component_labels = {'A: fPC1', 'B: fPC2', 'C: fPC3', 'D: fPC4'};
n_components = 4;

% 定义子图间距参数
vertical_spacing = 0.07;  % 垂直间距（占大子图高度的比例）
horizontal_spacing = 0.02;  % 垂直间距（占大子图高度的比例）
horizontal_left = 0.04;   % 左对齐起始位置（占大子图宽度的比例）
vertical_bottom = 0.06;  % 垂直间距（占大子图高度的比例）
sub_width_ratio = 0.07;    % 子图宽度（占大子图宽度的比例）
sub_height_ratio = 0.10;    % 子图宽度（占大子图宽度的比例）
lgd_left = 0.3;
lgd_bottom = 0.08;
lgd_width = 0.09;
lgd_height = 0.07;

% 生成四个大子图（2x2布局）
for comp = 1:n_components
    % 创建大子图容器
    h_k = mod((comp - 1), 2);
    v_k = floor((comp - 1) / 2);
    main_ax = subplot(2,2,comp);
    set(main_ax, 'Position', [0.05 + h_k * 0.42, 0.05 + (1 - v_k) * 0.42, 0.4, 0.4]); % 自定义位置和大小
    
    % 获取大子图位置 [left, bottom, width, height]
    main_pos = get(main_ax, 'Position');
    % disp(main_pos);
    
    % 大子图标题
    % title(component_labels{comp},'FontWeight', 'bold','FontSize', 14);
    
    reconstructed = alpha(comp) * S{comp};
    
    box(main_ax, 'on');                                 % 强制显示完整边框
    set(main_ax, ...
    'XTick', [], 'YTick', [], ...                   % 隐藏刻度
    'XColor', 'k', 'YColor', 'k', ...               % 设置坐标轴颜色为黑色（保留边框线）
    'XLabel', [], 'YLabel', []);                    % 隐藏坐标轴标签
    
    for dim = 1:n_dims
        % 计算绝对位置
        fcol = mod(dim-1, 4) + 1;
        frow = floor((dim-1)/4);
        absolute_left = main_pos(1) + horizontal_left + (fcol - 1) * (sub_width_ratio + horizontal_spacing);
        absolute_bottom = main_pos(2) + vertical_bottom + (1 - frow) * (sub_height_ratio + vertical_spacing);
        absolute_width = sub_width_ratio;
        absolute_height = sub_height_ratio;
        sub_pos = [absolute_left, absolute_bottom, absolute_width, absolute_height];

        % 创建小子图
        inner_ax = axes('Position', sub_pos);

        % 数据提取
        mean_trajectory = orign_data(:, dim);

        % 绘制双曲线
        h1 = plot(t, mean_trajectory, 'k-', 'LineWidth', 1.5); 
        hold on;
        % 计算主成分贡献（带符号）
        pc_contribution = reconstructed(:, dim);  % 从载荷矩阵获取方向

        % 绘制正负贡献（红色/蓝色）
        h2 = plot(t, mean_trajectory + pc_contribution, 'r--', 'LineWidth', 1.5);
        h3 = plot(t, mean_trajectory - pc_contribution, 'r:', 'LineWidth', 1.5);

        %% 坐标轴设置
        % 仅最下方小图显示x轴标签
        if dim == 1 || dim == 5
            xlabel('时间 (s)', 'FontSize', 10);
            ylabel('角度 (rad)', 'FontSize', 10);
        end
        
        title(['DoF ', num2str(dim)],'FontWeight', 'bold', 'FontSize', 12);

        % 统一设置y轴标签和范围
        mean_traj = mean_trajectory;
        pc_contrib = pc_contribution;
        all_data = [mean_traj; mean_traj+pc_contrib; mean_traj-pc_contrib];
        y_margin = 0.15*(max(all_data) - min(all_data));
        ylim([min(all_data)-y_margin, max(all_data)+y_margin]);
        box off;
        uistack(inner_ax, 'top');
    end
    lgd = legend([h1, h2, h3], {'Mean', ['Mean + α fPC',num2str(comp)], ['Mean - α fPC',num2str(comp)]}, ...
    'FontSize', 10, ...
    'Box', 'on', ...      
    'Position', [main_pos(1)+lgd_left, main_pos(2)+lgd_bottom, lgd_width, lgd_height]); % 归一化坐标系
end