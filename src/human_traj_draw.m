i = 1;
file_address = ['C:\Users\maircal\Desktop\毕设\UE_ADLdatabase-master\ADL017\ADL017FL',num2str(i),'angles.csv'];
[num,txt,raw] = xlsread(file_address);
rows = size(num, 1);
data = readcell(file_address);
data = data(2:rows+1, 5:11);           % 提取有效数据区域
numericArray = cell2mat(data);       % 转换为数值数组
numericArray = deg2rad(numericArray); % 转为弧度

%% 定义时间轴（假设原始时间为均匀分布）
original_time = linspace(0, 1, size(numericArray, 1))'; % 原始时间轴 [0,1] 区间
target_time = linspace(0, 1, 120)';                 % 目标时间轴（120点）

%% 初始化结果矩阵
joint_angles = zeros(120, 7);

%% 逐列进行三次样条插值
for col = 1:7
    joint_angles(:, col) = interp1(...
        original_time, ...       % 原始时间轴
        numericArray(:, col), ...    % 当前列数据
        target_time, ...         % 目标时间轴
        'spline', ...            % 三次样条插值
        'extrap' ...             % 允许外推（可选）
    );
end
path = joint_angles;
run('human_draw.m');