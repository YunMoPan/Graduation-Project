# Graduation-Project
这是一个毕业设计，关于七自由度仿生机械臂拟人运动规划

# Something you need know
* 代码部分基于MatlabR2021a平台
* 代码文件内容解释
  * *.mlapp appdesigner中的界面设计
  * Test.m为基础的机械臂展示，导入数据后即可展示动作动画
  * read_data.m为导入数据
  * fPCAs.m为主成分分析
  * RRT,kd_tree_RRT,RRT_Connect,RRT_pro,TRRT,TSQP为避障算法
  * deep_learning.m,简单深度学习优化
  * Comparison.m比较TSQP和深度学习优化算法
  * draw.m,draw2绘图，如<img width="382" alt="image" src="https://github.com/user-attachments/assets/ffd1c3b2-eb9f-4644-8060-507b50378a2d" />
* 数据来源于https://github.com/MiaHkin/UE_ADLdatabase
* 运动动画部分展示了实际仿真效果
* 由于本人精力和时间关系，代码中可修改数据文件来源以确保可以正常运行。具体文件为read_data.m等文件
* 部分代码可能在上传过程中没有调试过，所以需要稍微修改下
