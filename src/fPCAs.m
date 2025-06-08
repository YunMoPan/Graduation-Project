%% 数据准备
run('read_data.m');
[R, M, n_dims] = size(Q); % Q维度应为 R×M×7 (样本数×时间点数×关节维度)
if ~exist('s_max', 'var') 
   s_max = 3;
end
if s_max >= R             % 防止越界
    s_max = R;
end
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
sample_idx = 1;
alpha = score(sample_idx, 1:s_max); % 主成分权重

% 重构公式：q(t) = tilde_q + S0(t) + Σα_i*S_i(t)
reconstructed = repmat(S0, [1, 1]) + ...      % S0(t): M×7
               repmat(tilde_q, [M, 1]);      % tilde_q扩展为 M×7

for i = 1:s_max
    reconstructed = reconstructed + alpha(i) * S{i};
end