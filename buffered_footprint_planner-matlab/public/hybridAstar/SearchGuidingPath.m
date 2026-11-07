function SearchGuidingPath()

global params
% 1. 调用普通的二维 A* 算法（通常不考虑车身动力学，只考虑障碍物）
[x, y] = SearchAStarPath();    %-----（核心模块）-------------

fprintf('[Debug] SearchAStarPath returned: length(x)=%d, length(y)=%d\n', length(x), length(y));

if ~isempty(x)
    fprintf('[Debug] A* path start = (%.6f, %.6f)\n', x(1), y(1));
    fprintf('[Debug] A* path end   = (%.6f, %.6f)\n', x(end), y(end));
end

if isempty(x) || isempty(y)
    error('SearchGuidingPath:NoAStarPath', ...
        ['A* 未找到有效路径。\n' ...
         '建议检查：起终点合法性、障碍物栅格化、地图边界、障碍膨胀、A* 搜索逻辑。']);
end


% 2. 路径重采样：将 A* 生成的、疏密不均的点变成等间距的、点数固定的点
[params.guiding_path.x, params.guiding_path.y] = ResamplePath(x, y);

%-----------得到了采样点集（x,y），通过artan（dx/dy）得到航向角theta-----------------
% 3. 初始化航向角数组
params.guiding_path.theta = zeros(1, length(params.guiding_path.x));
params.guiding_path.theta(1) = params.task.theta0; % 起点继承任务设定的角度

% 4. 几何角度补全：利用前后两点的坐标差计算当前点的朝向（切线方向）
x = params.guiding_path.x;
y = params.guiding_path.y;
for ii = 2 : length(params.guiding_path.x)
    % 使用 atan2(dx, dy) 计算两点连线与 Y 轴的夹角
    params.guiding_path.theta(ii) = atan2(x(ii) - x(ii-1), y(ii) - y(ii-1));
end

fprintf('[Debug] SearchAStarPath success');

end

%% 重采样
function [x_full, y_full] = ResamplePath(x, y)
if isempty(x) || isempty(y)
    error('ResamplePath:EmptyInput', ...
        'ResamplePath 输入为空：SearchGuidingPath 未找到有效引导路径。');
end
global params
x_full = [];
y_full = [];

% ----------第一阶段：加密（插值）
for ii = 2 : length(x)
    % 计算相邻两点间的欧式距离，并乘以因子 nfe，决定这一段要插多少个点
    Nsp = round(norm([x(ii) - x(ii-1), y(ii) - y(ii-1)]) * params.ha.nfe);  % round为四舍五入函数，norm用于计算两点间距离，求该向量的二范数
    
    % 在这两点之间进行线性插值
    temp = linspace(x(ii-1), x(ii), Nsp);
    x_full = [x_full, temp(1, 1 : (Nsp - 1))]; % 存入 x 序列
    temp = linspace(y(ii-1), y(ii), Nsp);
    y_full = [y_full, temp(1, 1 : (Nsp - 1))]; % 存入 y 序列
end
x_full = [x_full, x(end)]; % 补上最后一个终点
y_full = [y_full, y(end)];

% 第二阶段：匀化（重采样）
% 无论之前的路径有多长，统一强制提取出 params.ha.nfe 个点
ind = round(linspace(1, length(x_full), params.ha.nfe));
x_full = x_full(ind);
y_full = y_full(ind);
end