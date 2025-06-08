% 障碍物定义 [x,y,z,radius]
obstacles = [-0.1 -0.2 -0.1 0.05;
             0 -0.2 -0.4 0.05;
             0.1 -0.3 0.1 0.05];

figure(1);
robot.plot(q_start); 
hold on;

% 设置颜色映射（从红色到橙色渐变）
color_map = [linspace(1, 0.8, size(obstacles,1))', ...  % R
             linspace(0, 0.2, size(obstacles,1))', ...  % G
             zeros(size(obstacles,1), 1)];              % B & Alpha

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
view(45, 30);           % 调整视角
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Obstacle Visualization with Spheres');
drawnow;