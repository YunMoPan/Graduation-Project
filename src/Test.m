% 障碍物定义 [x,y,z,radius]
obstacles = [0.3 0.2 0.3 0.1;
             0.1 -0.3 0.2 0.15]; 
figure;
[x,y,z] = sphere(20);
[num,~] = size(obstacles);
for i = 1:num
    ob = transpose(obstacles(i,:));
    surf(x*ob(4) + ob(1),...
         y*ob(4) + ob(2),...
         z*ob(4) + ob(3),...
         'FaceAlpha',0.3);
end
hold on;