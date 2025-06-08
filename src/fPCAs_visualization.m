%% fPCAs
run('fPCAs.m');

%% 多维度对比可视化（修正子图布局）
joint_names = {'肩水平内收', '肩屈伸', '肩部内外旋转', '肘关节屈伸', '前臂旋前旋后', '腕屈伸', '腕桡尺偏斜'};
figure('Position', [100, 100, 1400, 800]);

% 创建紧凑布局
tLayout = tiledlayout(2, 4, 'TileSpacing', 'tight', 'Padding', 'tight');

% 绘制前7个子图
for dim = 1:n_dims
    nexttile; % 自动定位
    
    % 数据提取
    orig_data = squeeze(Q(sample_idx, :, dim)); 
    
    % 绘制双曲线
    h1 = plot(t, orig_data, 'b-', 'LineWidth', 1.5); 
    hold on;
    h2 = plot(t, reconstructed(:, dim), 'r--', 'LineWidth', 1.5);
    
    % 子图标注
    title(joint_names{dim}, 'FontSize', 9); % 调小标题字号
    xlabel('时间 (s)', 'FontSize', 8);
    ylabel('角度 (rad)', 'FontSize', 8);
    grid on;
    
    % 动态调整Y轴范围
    y_margin = 0.15 * range(orig_data); % 根据数据范围自动计算边距
    ylim([min(orig_data)-y_margin, max(orig_data)+y_margin]);
end

% 在右下角添加共享图例 (精准定位)
nexttile(8);
axis off;
lgd = legend([h1, h2], {'原始数据', '重构数据'}, ...
    'FontSize', 9, ...
    'Box', 'off', ...       % 去掉图例边框
    'Position', [0.82 0.12 0.12 0.08]); % 归一化坐标系

% 设置总标题.
title(tLayout, ['样本 ', num2str(sample_idx), ' 轨迹重构对比 (s_{max}=', num2str(s_max), ')'], ...
    'FontSize', 11, ...
    'FontWeight', 'bold', ...
    'Interpreter', 'none'); % 防止s_max中的下划线被识别为下标