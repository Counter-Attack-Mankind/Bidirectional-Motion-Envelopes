function report = CheckEFReverseCollision(show_plot)
% CheckEFReverseCollision
% ------------------------------------------------------------
% 使用 SAT 分离轴定理检查原始优化采样点的具身盒子与障碍物多边形是否碰撞
%
% 重点定位：
%   1) 哪些采样点发生碰撞
%   2) 碰撞是否主要发生在后退段
%   3) 是否集中在换向附近
%
% 输入:
%   show_plot  : 是否画图标注（true/false），默认 true
%
% 输出:
%   report     : 结构体，包含所有碰撞明细
% ------------------------------------------------------------

global params

if nargin < 1
    show_plot = true;
end

%% ========== 基本读取 ==========
assert(isfield(params, 'ef'), 'params.ef 不存在');
assert(isfield(params, 'environment'), 'params.environment 不存在');
assert(isfield(params.environment, 'obs'), 'params.environment.obs 不存在');

x     = params.ef.x(:);
y     = params.ef.y(:);
theta = params.ef.theta(:);

up    = params.ef.up(:);
down  = params.ef.down(:);
left  = params.ef.left(:);
right = params.ef.right(:);

Nstate = length(x);

if length(up) == Nstate - 1
    Nbox = Nstate - 1;
elseif length(up) == Nstate
    Nbox = Nstate;
else
    error('up/down/left/right 长度与状态点数不匹配');
end

has_v  = isfield(params.ef, 'v')  && ~isempty(params.ef.v);
has_dt = isfield(params.ef, 'dt') && ~isempty(params.ef.dt);
has_s  = isfield(params.ef, 's')  && ~isempty(params.ef.s);
has_k  = isfield(params.ef, 'k')  && ~isempty(params.ef.k);
has_a  = isfield(params.ef, 'a')  && ~isempty(params.ef.a);

if has_v,  v  = params.ef.v(:);  else, v  = nan(Nstate,1); end
if has_dt, dt = params.ef.dt(:); else, dt = nan(Nbox,1);   end
if has_s,  s  = params.ef.s(:);  else, s  = nan(Nbox,1);   end
if has_k,  k  = params.ef.k(:);  else, k  = nan(Nbox,1);   end
if has_a,  a  = params.ef.a(:);  else, a  = nan(Nstate,1); end

%% ========== 车辆参数 ==========
if isfield(params.vehicle, 'LF')
    LF = params.vehicle.LF;
elseif isfield(params.vehicle, 'lf')
    LF = params.vehicle.lf;
else
    error('params.vehicle 中找不到 LF 或 lf');
end

assert(isfield(params.vehicle, 'lr'),  'params.vehicle.lr 不存在');
assert(isfield(params.vehicle, 'hlb'), 'params.vehicle.hlb 不存在');

lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;
lb  = 2 * hlb;

%% ========== 初始化输出 ==========
report = struct();
report.total_boxes = Nbox;
report.collision_count = 0;
report.forward_collision_count = 0;
report.reverse_collision_count = 0;
report.switch_near_collision_count = 0;
report.items = [];

%% ========== 可选画图 ==========
if show_plot
    fig = figure('Name','Check EF Reverse Collision (SAT)','Color','w');
    ax = axes(fig); hold(ax,'on'); grid(ax,'minor'); box(ax,'on'); axis(ax,'equal');

    if isfield(params.environment,'xmin') && isfield(params.environment,'xmax') ...
            && isfield(params.environment,'ymin') && isfield(params.environment,'ymax')
        axis(ax, [params.environment.xmin, params.environment.xmax, ...
                  params.environment.ymin, params.environment.ymax]);
    end

    xlabel(ax,'x / m');
    ylabel(ax,'y / m');
    title(ax,'Collision Diagnosis of Original EF Boxes (SAT)');

    for nn = 1:length(params.environment.obs)
        obs = params.environment.obs(nn);
        fill(ax, obs.x, obs.y, [0.8 0.8 0.8], ...
            'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
    end

    plot(ax, x, y, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(ax, x, y, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k', ...
        'HandleVisibility', 'off');
end

%% ========== 主循环 ==========
item_idx = 0;

for i = 1:Nbox

    % ---- 盒子四角，严格按 NLP 公式 ----
    c = cos(theta(i));
    snt = sin(theta(i));

    AX = x(i) + (LF + up(i))   * c - (hlb + left(i))  * snt;
    BX = x(i) + (LF + up(i))   * c + (hlb + right(i)) * snt;
    CX = x(i) - (lr + down(i)) * c + (hlb + right(i)) * snt;
    DX = x(i) - (lr + down(i)) * c - (hlb + left(i))  * snt;

    AY = y(i) + (LF + up(i))   * snt + (hlb + left(i))  * c;
    BY = y(i) + (LF + up(i))   * snt - (hlb + right(i)) * c;
    CY = y(i) - (lr + down(i)) * snt - (hlb + right(i)) * c;
    DY = y(i) - (lr + down(i)) * snt + (hlb + left(i))  * c;

    boxX = [AX BX CX DX];
    boxY = [AY BY CY DY];
    box_poly = [AX AY; BX BY; CX CY; DX DY];

    box_area_formula  = (LF + lr + up(i) + down(i)) * (lb + left(i) + right(i));
    box_area_shoelace = polyarea(boxX, boxY);

    % ---- 判断方向 ----
    seg_dir = "unknown";
    if has_s && i <= length(s)
        if s(i) > 1e-10
            seg_dir = "forward";
        elseif s(i) < -1e-10
            seg_dir = "reverse";
        else
            seg_dir = "zero";
        end
    elseif has_v
        if v(i) > 1e-10
            seg_dir = "forward";
        elseif v(i) < -1e-10
            seg_dir = "reverse";
        else
            seg_dir = "zero";
        end
    end

    % ---- 是否在换向附近 ----
    near_switch = false;
    if has_v && i < length(v)
        if sign_eps(v(i)) ~= sign_eps(v(i+1))
            near_switch = true;
        elseif i > 1 && sign_eps(v(i-1)) ~= sign_eps(v(i))
            near_switch = true;
        end
    end

    % ---- 画盒子 ----
    if show_plot
        if seg_dir == "reverse"
            edge_color = [0.85 0.15 0.15];
            face_color = [1.00 0.55 0.55];
        elseif seg_dir == "forward"
            edge_color = [0.10 0.35 0.95];
            face_color = [0.50 0.75 1.00];
        else
            edge_color = [0.55 0.10 0.75];
            face_color = [0.85 0.65 1.00];
        end

        patch(ax, [boxX boxX(1)], [boxY boxY(1)], face_color, ...
            'FaceAlpha', 0.10, 'EdgeColor', edge_color, ...
            'LineWidth', 0.7, 'HandleVisibility', 'off');
    end

    % ---- 使用 SAT 检查盒子与障碍物是否碰撞 ----
    this_box_has_collision = false;

    for nn = 1:length(params.environment.obs)
        obs = params.environment.obs(nn);
        obs_poly = [obs.x(:), obs.y(:)];

        is_collide = SAT_PolygonCollision(box_poly, obs_poly);

        if ~is_collide
            continue;
        end

        this_box_has_collision = true;

        % 为了输出更细的明细，再找"哪些障碍物顶点在盒子内/边上"
        Px_all = obs.x(:);
        Py_all = obs.y(:);
        [in_flag_all, on_flag_all] = inpolygon(Px_all, Py_all, [boxX boxX(1)], [boxY boxY(1)]);

        hit_idx = find(in_flag_all | on_flag_all);

        % 如果没有顶点落入，也可能是边边相交型碰撞
        if isempty(hit_idx)
            item_idx = item_idx + 1;

            item = struct();
            item.box_index = i;
            item.obs_index = nn;
            item.vertex_index = NaN;
            item.Px = NaN;
            item.Py = NaN;

            item.direction = char(seg_dir);
            item.near_switch = near_switch;

            item.v_i = get_safe(v, i);
            item.v_ip1 = get_safe(v, i+1);
            item.a_i = get_safe(a, i);
            item.dt_i = get_safe(dt, i);
            item.s_i = get_safe(s, i);
            item.k_i = get_safe(k, i);

            item.up_i = up(i);
            item.down_i = down(i);
            item.left_i = left(i);
            item.right_i = right(i);

            item.area_sum = NaN;
            item.box_area_formula = box_area_formula;
            item.box_area_shoelace = box_area_shoelace;
            item.margin = NaN;
            item.inpolygon_in = false;
            item.inpolygon_on = false;
            item.collision_type = 'SAT_edge_or_overlap';

            item.boxX = boxX;
            item.boxY = boxY;

            report.items = [report.items; item]; %#ok<AGROW>

            if show_plot
                cx = mean(boxX);
                cy = mean(boxY);
                plot(ax, cx, cy, 'rs', 'MarkerSize', 8, ...
                    'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
                text(cx, cy, sprintf('  i=%d,obs=%d,SAT', i, nn), ...
                    'Color', [0.75 0 0], 'FontSize', 8, 'FontWeight', 'bold');
            end
        else
            for jj = hit_idx(:)'
                Px = Px_all(jj);
                Py = Py_all(jj);

                % 为兼容你原先报告格式，这里保留面积量
                A1 = 0.5 * abs((AX - Px)*(BY - Py) - (AY - Py)*(BX - Px));
                A2 = 0.5 * abs((BX - Px)*(CY - Py) - (BY - Py)*(CX - Px));
                A3 = 0.5 * abs((CX - Px)*(DY - Py) - (CY - Py)*(DX - Px));
                A4 = 0.5 * abs((DX - Px)*(AY - Py) - (DY - Py)*(AX - Px));
                area_sum = A1 + A2 + A3 + A4;
                margin = area_sum - (box_area_formula + 0.1);

                item_idx = item_idx + 1;

                item = struct();
                item.box_index = i;
                item.obs_index = nn;
                item.vertex_index = jj;
                item.Px = Px;
                item.Py = Py;

                item.direction = char(seg_dir);
                item.near_switch = near_switch;

                item.v_i = get_safe(v, i);
                item.v_ip1 = get_safe(v, i+1);
                item.a_i = get_safe(a, i);
                item.dt_i = get_safe(dt, i);
                item.s_i = get_safe(s, i);
                item.k_i = get_safe(k, i);

                item.up_i = up(i);
                item.down_i = down(i);
                item.left_i = left(i);
                item.right_i = right(i);

                item.area_sum = area_sum;
                item.box_area_formula = box_area_formula;
                item.box_area_shoelace = box_area_shoelace;
                item.margin = margin;
                item.inpolygon_in = in_flag_all(jj);
                item.inpolygon_on = on_flag_all(jj);
                item.collision_type = 'SAT_vertex_inside';

                item.boxX = boxX;
                item.boxY = boxY;

                report.items = [report.items; item]; %#ok<AGROW>

                if show_plot
                    plot(ax, Px, Py, 'rp', 'MarkerSize', 11, ...
                        'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
                    text(Px, Py, sprintf('  i=%d,obs=%d,vtx=%d', i, nn, jj), ...
                        'Color', [0.75 0 0], 'FontSize', 8, ...
                        'FontWeight', 'bold');
                end
            end
        end
    end

    if this_box_has_collision
        report.collision_count = report.collision_count + 1;

        if seg_dir == "forward"
            report.forward_collision_count = report.forward_collision_count + 1;
        elseif seg_dir == "reverse"
            report.reverse_collision_count = report.reverse_collision_count + 1;
        end

        if near_switch
            report.switch_near_collision_count = report.switch_near_collision_count + 1;
        end
    end
end

%% ========== 终端输出 ==========
fprintf('\n================ Collision Diagnosis Report ================\n');
fprintf('总盒子数                    : %d\n', report.total_boxes);
fprintf('发生碰撞的盒子数            : %d\n', report.collision_count);
fprintf('前进段碰撞盒子数            : %d\n', report.forward_collision_count);
fprintf('后退段碰撞盒子数            : %d\n', report.reverse_collision_count);
fprintf('换向附近碰撞盒子数          : %d\n', report.switch_near_collision_count);
fprintf('碰撞明细条目数              : %d\n', length(report.items));
fprintf('============================================================\n\n');

if ~isempty(report.items)
    fprintf('前 20 条碰撞明细如下：\n');
    nshow = min(20, length(report.items));
    for t = 1:nshow
        it = report.items(t);
        fprintf(['#%02d | box=%d, obs=%d, vtx=%g | dir=%s | nearSwitch=%d | ' ...
                 'v(i)=%.4f, v(i+1)=%.4f, dt=%.4f, s=%.4f, k=%.4f | ' ...
                 'up=%.4f, down=%.4f, left=%.4f, right=%.4f | type=%s | margin=%.6f\n'], ...
                 t, it.box_index, it.obs_index, it.vertex_index, ...
                 it.direction, it.near_switch, ...
                 it.v_i, it.v_ip1, it.dt_i, it.s_i, it.k_i, ...
                 it.up_i, it.down_i, it.left_i, it.right_i, ...
                 it.collision_type, it.margin);
    end
else
    fprintf('未检测到具身盒子与障碍物的 SAT 碰撞。\n');
end

end


%% ==================== SAT 辅助函数 ====================

function is_collide = SAT_PolygonCollision(poly1, poly2)
% poly1, poly2: N×2, M×2
% true = 碰撞，false = 无碰撞

is_collide = true;

axes = [GetAxesFromPolygon(poly1);
        GetAxesFromPolygon(poly2)];

eps_sat = 1e-10;

for k = 1:size(axes,1)
    axis_k = axes(k,:);
    nrm = norm(axis_k);
    if nrm < 1e-12
        continue;
    end
    axis_k = axis_k / nrm;

    proj1 = poly1 * axis_k';
    proj2 = poly2 * axis_k';

    if max(proj1) < min(proj2) - eps_sat || max(proj2) < min(proj1) - eps_sat
        is_collide = false;
        return;
    end
end
end

function axes = GetAxesFromPolygon(poly)
n = size(poly,1);
axes = zeros(n,2);

for i = 1:n
    p1 = poly(i,:);
    if i < n
        p2 = poly(i+1,:);
    else
        p2 = poly(1,:);
    end

    edge = p2 - p1;
    axes(i,:) = [-edge(2), edge(1)];
end
end


%% ==================== 其他辅助函数 ====================

function y = get_safe(vec, idx)
if isempty(vec) || idx < 1 || idx > length(vec)
    y = nan;
else
    y = vec(idx);
end
end

function s = sign_eps(x)
eps0 = 1e-10;
if x > eps0
    s = 1;
elseif x < -eps0
    s = -1;
else
    s = 0;
end
end