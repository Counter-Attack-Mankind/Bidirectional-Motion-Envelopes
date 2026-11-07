function LoadTask(task_id)      % 任务环境的加载
global params


% 当前文件目录（Utilities）
current_dir = fileparts(mfilename('fullpath'));
% 项目根目录（上一级）
project_root = fileparts(current_dir);
% 数据目录
data_dir = fullfile(project_root, 'Environment', 'real');

task_source = strtrim(getenv('EXPERIMENT_TASK_SOURCE'));
if isempty(task_source)
    task_source = 'real';
end
if ~any(strcmpi(task_source, {'real','Data_test'}))
    error('LoadTask:InvalidSource', 'EXPERIMENT_TASK_SOURCE must be real or Data_test, not %s.',task_source);
end


% 构造文件路径
file_path = fullfile(data_dir, [num2str(task_id), '.mat']);
% 检查文件是否存在
if ~isfile(file_path)
    error('任务文件不存在: %s', file_path);
end
% 加载
load(file_path);


params.environment.obs = obs;   % 存储障碍物顶点坐标（多边形）
params.task.x0 = x0;
params.task.y0 = y0;
params.task.theta0 = theta0;
params.task.xf = xf;
params.task.yf = yf;
params.task.thetaf = thetaf;

params.environment.num_obs = size(obs, 2);  % 障碍物的个数为2
CreateDilatedCostmap();     % 构建膨胀代价地图
CreateDilatedGrids();       % 构建离散化栅格点集合
end

%%  创建膨胀代价地图
function CreateDilatedCostmap()
global params
xmin = params.environment.xmin;   % 地图左边界
ymin = params.environment.ymin;   % 地图下边界
resolution_x = params.ha.dx;      % 栅格横向分辨率（单位：米）
resolution_y = params.ha.dy;      % 栅格纵向分辨率

% 初始化两个矩阵：costmap记录是否被占据，potential_field记录障碍物附近的"危险程度"
costmap = zeros(params.ha.nx, params.ha.ny); 
params.potential_field = zeros(params.ha.nx, params.ha.ny);

% -----------------------从几何障碍物到栅格地图的"离散化"映射------------------------
for ii = 1 : params.environment.num_obs     % 对障碍物进行处理
    vx = params.environment.obs(ii).x; % 当前障碍物所有顶点的X坐标
    vy = params.environment.obs(ii).y; % 当前障碍物所有顶点的Y坐标
    
    % AABB（轴对齐包围盒）优化：只计算障碍物所在的矩形区域，提高效率
    x_lb = min(vx); x_ub = max(vx);  % 将不规则的多边形障碍物------> AABB（轴对称的包围盒），外接矩形
    y_lb = min(vy); y_ub = max(vy);
    
    % 将物理坐标转换为矩阵的行列索引
    [Nmin_x, Nmin_y] = ConvertXYToIndex(x_lb, y_lb);
    [Nmax_x, Nmax_y] = ConvertXYToIndex(x_ub, y_ub);
    
    % 遍历AABB包围盒障碍物所有的点（通过双循环）
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj, kk) == 1), continue; end % 如果已经是障碍物，跳过
            
            % 将当前网格索引转回物理坐标，用于判断点是否在多边形内
            cur_x = xmin + (jj - 1) * resolution_x;
            cur_y = ymin + (kk - 1) * resolution_y;
            
            % 判断该栅格中心点是否在障碍物多边形内部
            if (inpolygon(cur_x, cur_y, vx, vy) == 1)
                costmap(jj, kk) = 1;         % 标记为被占据
                UpdatePotentialField(jj, kk); % 更新该障碍物点周边的势场
            end
        end
    end
end

% 图像形态学膨胀：这是为了给车辆留出"安全边距"，length_unit 是栅格的平均尺寸
length_unit = 0.5 * (resolution_x + resolution_y);

% 创建圆形膨胀算子。半径 = (车宽的一半 * 1.5) / 栅格尺寸
basic_elem = strel('disk', 1 + ceil((params.vehicle.hlb * 1.3) / length_unit)); % 调节膨胀大小的关键

params.dialated_map = imdilate(costmap, basic_elem); % 执行膨胀
params.basic_map = costmap; % 原始未膨胀的地图
% 归一化势场，使其最大值为1
params.potential_field = params.potential_field ./ max(max(params.potential_field));
end

%% 米与矩阵行列的转换
function [ind1,ind2] = ConvertXYToIndex(x, y)   
global params
ind1 = ceil((x - params.environment.xmin) / params.ha.dx) + 1;
ind2 = ceil((y - params.environment.ymin) / params.ha.dy) + 1;
ind1 = max(1, min(ind1, params.ha.nx));
ind2 = max(1, min(ind2, params.ha.ny));
end

%% 提取膨胀的点云
function CreateDilatedGrids()   
global params
[ii, jj] = find(params.dialated_map == 1);
params.dialated_xy_grids_cell.x = params.environment.xmin + (ii-1) .* params.ha.dx;
params.dialated_xy_grids_cell.y = params.environment.ymin + (jj-1) .* params.ha.dy;
end

%% 创建人工势场来引导避障，使得车辆尽可能走在中间路线，而非旁边
function UpdatePotentialField(jj, kk)
global params
ind_x_lb = max(min(jj - 5, params.ha.nx), 1);
ind_x_ub = max(min(jj + 5, params.ha.nx), 1);
ind_y_lb = max(min(kk - 5, params.ha.ny), 1);
ind_y_ub = max(min(kk + 5, params.ha.ny), 1);
for i = ind_x_lb : ind_x_ub
    for j = ind_y_lb : ind_y_ub
        distance = hypot(i - jj, j - kk);   % 计算当前格点到障碍物格点的欧几里得距离
        params.potential_field(i,j) = params.potential_field(i,j) + exp(-0.01 * distance);  % 势场函数：距离越近，增加的数值越大（指数衰减）
    end
end
end
