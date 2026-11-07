function report =CheckWrittenInitialGuessForNLP(matfile, tol, doPlot)
% CheckInitialGuessAgainstCurrentNLP
% 按当前 NLP.mod 与 WriteEFInitialGuess 严格对齐，检查初始解质量
%
% 用法：
%   report = CheckInitialGuessAgainstCurrentNLP();
%   report = CheckInitialGuessAgainstCurrentNLP('written_initial_guess_data.mat');
%   report = CheckInitialGuessAgainstCurrentNLP('written_initial_guess_data.mat', 1e-8, true);
%
% 检查内容：
%   1) 时间变量与 tf
%   2) 车辆运动学离散约束
%   3) 首末边界约束
%   4) v/a/phy/w 边界
%   5) 具身变量定义约束
%   6) 具身三约束
%   7) 具身盒子顶点关系
%   8) 角点边界约束
%   9) 三角面积避障约束（严格按当前 mod：三角/四边形）
%   10) 可视化总览 + 各检查图
%
% 说明：
%   - 当前 NLP.mod 仅支持三角形/四边形障碍物。
%   - 三角形在 WriteEFInitialGuess 中被补成伪四边形 P1,P2,P3,P3。
%   - Area 用的是 min(area+0.02, area*1.02)，本函数与之保持一致。
%
% 作者建议：
%   若 IPOPT 出现 restoration failed / locally infeasible，
%   先看：
%     report.summary
%     report.ineq.eq_PPPoutsideABCD
%     report.eq.DIFF_dvdt
%     report.eq.DIFF_dphydt
%     report.ineq.EF_forward_cond1
%     report.ineq.EF_reverse_cond1

if nargin < 1 || isempty(matfile)
    matfile = 'written_initial_guess_data.mat';
end
if nargin < 2 || isempty(tol)
    tol = 1e-8;
end
if nargin < 3 || isempty(doPlot)
    doPlot = true;
end

S = load(matfile);
if ~isfield(S, 'data')
    error('文件 %s 中未找到变量 data', matfile);
end
D = S.data;

%========================
% 读取数据
%========================
x     = D.x(:);
y     = D.y(:);
theta = D.theta(:);
v     = D.v(:);
a     = D.a(:);
phy   = D.phy(:);
w     = D.w(:);

dt     = D.dt(:);
tf     = D.tf;
k      = D.kappa(:);
s      = D.s(:);
splus  = D.splus(:);
sminus = D.sminus(:);

up     = D.up(:);
down   = D.down(:);
left   = D.left(:);
right  = D.right(:);

AX = D.AX(:); AY = D.AY(:);
BX = D.BX(:); BY = D.BY(:);
CX = D.CX(:); CY = D.CY(:);
DX = D.DX(:); DY = D.DY(:);

meta = D.meta;

Nfe = meta.Nfe;
Nobs = meta.Nobs;

lw      = meta.lw;
LF      = meta.LF;
lr      = meta.lr;
hlb     = meta.hlb;
lb      = 2 * hlb;

v_max   = meta.v_max;
a_max   = meta.a_max;
phy_max = meta.phy_max;
w_max   = meta.w_max;
max_dt  = meta.max_dt;
min_dt  = meta.min_dt;
obs = meta.obs;

% 当前 NLP.mod 角点边界是写死 0~30
xmin_box = 0; xmax_box = 30;
ymin_box = 0; ymax_box = 30;

%========================
% 维度检查
%========================
assert(length(x)     == Nfe,   'x 长度错误');
assert(length(y)     == Nfe,   'y 长度错误');
assert(length(theta) == Nfe,   'theta 长度错误');
assert(length(v)     == Nfe,   'v 长度错误');
assert(length(a)     == Nfe,   'a 长度错误');
assert(length(phy)   == Nfe,   'phy 长度错误');
assert(length(w)     == Nfe,   'w 长度错误');

assert(length(dt)     == Nfe-1, 'dt 长度错误');
assert(length(k)      == Nfe-1, 'k 长度错误');
assert(length(s)      == Nfe-1, 's 长度错误');
assert(length(splus)  == Nfe-1, 'splus 长度错误');
assert(length(sminus) == Nfe-1, 'sminus 长度错误');
assert(length(up)     == Nfe-1, 'up 长度错误');
assert(length(down)   == Nfe-1, 'down 长度错误');
assert(length(left)   == Nfe-1, 'left 长度错误');
assert(length(right)  == Nfe-1, 'right 长度错误');

%========================
% 报告容器,三类模板
%========================
report = struct();
report.meta = meta;
report.tol = tol;
report.eq = struct();
report.ineq = struct();
report.plotdata = struct();

    function S1 = makeEqReport(res, name)   % 等式比较-看差值残差（residual=当前初始解中的值−按约束公式重算出来的值）
        S1 = struct();
        S1.name = name;
        S1.max_abs = max(abs(res));
        S1.mean_abs = mean(abs(res));
        S1.num_bad = nnz(abs(res) > tol);
        S1.bad_idx = find(abs(res) > tol);
        S1.residual = res(:);
    end

    function S1 = makeIneqUpper(val, ub, name)  %  上界不等式比较：看"违反量"
        vio = max(val - ub, 0);
        slack = ub - val;
        S1 = struct();
        S1.name = name;
        S1.max_violation = max(vio);
        S1.mean_violation = mean(vio);
        S1.min_slack = min(slack);
        S1.num_bad = nnz(vio > tol);
        S1.bad_idx = find(vio > tol);
        S1.violation = vio(:);
        S1.slack = slack(:);
    end

    function S1 = makeRangeReport(val, lb_, ub_, name)  % 区间边界比较：看是否落在 [lb, ub]
        vio_low  = max(lb_ - val, 0);
        vio_high = max(val - ub_, 0);
        vio = max(vio_low, vio_high);
        S1 = struct();
        S1.name = name;
        S1.min_val = min(val);
        S1.max_val = max(val);
        S1.max_violation = max(vio);
        S1.mean_violation = mean(vio);
        S1.num_bad = nnz(vio > tol);
        S1.bad_idx = find(vio > tol);
        S1.violation = vio(:);
    end

%========================
% 1) tf / dt
%========================
report.eq.define_tf = makeEqReport(tf - sum(dt), 'define_tf');      % tf 是否等于所有 dt 之和
report.ineq.time_bound1 = makeRangeReport(dt, min_dt, max_dt, 'time_bound1');   % 每个 dt(i) 是否在允许范围 [min_dt, max_dt]
report.ineq.time_bound2 = makeIneqUpper(tf, 30, 'time_bound2');     % tf 是否超过 30

%========================
% 2) 车辆运动学离散约束
%========================
res_dx   = zeros(Nfe-1,1);          
res_dy   = zeros(Nfe-1,1);
res_dv   = zeros(Nfe-1,1);
res_dth  = zeros(Nfe-1,1);
res_dphy = zeros(Nfe-1,1);

for i = 1:Nfe-1
    res_dy(i)   = y(i+1)     - (y(i)     + v(i) * dt(i) * sin(theta(i)));   %比较初始解中存的 x(i+1)和根据 x(i), v(i), dt(i), theta(i) 按离散动力学公式推出来的"理论下一步位置"
    res_dv(i)   = v(i+1)     - (v(i)     + dt(i) * a(i));
    res_dth(i)  = theta(i+1) - (theta(i) + dt(i) * v(i) * tan(phy(i)) / lw);
    res_dphy(i) = phy(i+1)   - (phy(i)   + dt(i) * w(i));
end

report.eq.DIFF_dxdt     = makeEqReport(res_dx,   'DIFF_dxdt');
report.eq.DIFF_dydt     = makeEqReport(res_dy,   'DIFF_dydt');
report.eq.DIFF_dvdt     = makeEqReport(res_dv,   'DIFF_dvdt');
report.eq.DIFF_dthetadt = makeEqReport(res_dth,  'DIFF_dthetadt');
report.eq.DIFF_dphydt   = makeEqReport(res_dphy, 'DIFF_dphydt');

%========================
% 3) 两点边值约束 
%========================
bc = [];
bc_name = {};

% 边界条件比较：首末节点是否和任务要求一致
appendBC(x(1)     - meta.x0,     'Init_X');
appendBC(y(1)     - meta.y0,     'Init_Y');
appendBC(theta(1) - meta.theta0, 'Init_Theta');

appendBC(x(Nfe)     - meta.xf,     'End_X');
appendBC(y(Nfe)     - meta.yf,     'End_Y');
appendBC(theta(Nfe) - meta.thetaf, 'End_Theta');

appendBC(w(1),   'Init_W');
appendBC(a(1),   'Init_A');
appendBC(phy(1), 'Init_Phy');
appendBC(v(1),   'Init_v');

appendBC(w(Nfe),   'End_W');
appendBC(a(Nfe),   'End_A');
appendBC(phy(Nfe), 'End_Phy');
appendBC(v(Nfe),   'End_v');

report.eq.boundary_all = makeEqReport(bc, 'BoundaryAll');
report.eq.boundary_names = bc_name;

%========================
% 4) 节点变量边界
%========================

%节点变量边界比较：变量是否超范围
report.ineq.Bonds_v   = makeRangeReport(v,   -v_max,   v_max,   'Bonds_v');
report.ineq.Bonds_a   = makeRangeReport(a,   -a_max,   a_max,   'Bonds_a');
report.ineq.Bonds_phy = makeRangeReport(phy, -phy_max, phy_max, 'Bonds_phy');
report.ineq.Bonds_w   = makeRangeReport(w,   -w_max,   w_max,   'Bonds_w');

%========================
% 5) 具身变量定义约束
%========================
k_re      = tan(phy(1:Nfe-1)) / lw;
s_re      = v(1:Nfe-1) .* dt;
splus_re  = max(s_re, 0);
sminus_re = max(-s_re, 0);

up_re     = zeros(Nfe-1,1);
down_re   = zeros(Nfe-1,1);
left_re   = zeros(Nfe-1,1);
right_re  = zeros(Nfe-1,1);

for i = 1:Nfe-1
    kk = k_re(i);
    sp = splus_re(i);
    sm = sminus_re(i);

    up_re(i)   = sp + hlb * abs(kk) * sp;
    down_re(i) = sm + hlb * abs(kk) * sm;

    left_re(i) = max(-lr * kk * sp, (LF + 0.5 * sp) * kk * sp) ...
               + max(-LF * kk * sm, (lr + 0.5 * sm) * kk * sm);

    right_re(i)= max( lr * kk * sp, -(LF + 0.5 * sp) * kk * sp) ...
               + max( LF * kk * sm, -(lr + 0.5 * sm) * kk * sm);
end

report.eq.define_kappa  = makeEqReport(k - k_re, 'define_kappa');
report.eq.define_s      = makeEqReport(s - s_re, 'define_s');
report.eq.define_splus  = makeEqReport(splus - splus_re, 'define_splus');
report.eq.define_sminus = makeEqReport(sminus - sminus_re, 'define_sminus');
report.eq.define_up     = makeEqReport(up - up_re, 'define_up');
report.eq.define_down   = makeEqReport(down - down_re, 'define_down');
report.eq.define_left   = makeEqReport(left - left_re, 'define_left');
report.eq.define_right  = makeEqReport(right - right_re, 'define_right');

%========================
% 6) 具身三约束
%========================
ef_arc = abs(k) .* (splus + sminus);
ef_f1  = (1 + hlb * abs(k)) .* tan(abs(k) .* splus)  - lr * abs(k);
ef_f2  = abs(k) * LF .* tan(abs(k) .* splus)         - (1 + hlb * abs(k));
ef_r1  = (1 + hlb * abs(k)) .* tan(abs(k) .* sminus) - LF * abs(k);
ef_r2  = abs(k) * lr .* tan(abs(k) .* sminus)        - (1 + hlb * abs(k));

report.ineq.EF_arc_bound     = makeIneqUpper(ef_arc, 1.5708 * ones(size(ef_arc)), 'EF_arc_bound');
report.ineq.EF_forward_cond1 = makeIneqUpper(ef_f1,  zeros(size(ef_f1)), 'EF_forward_cond1');
report.ineq.EF_forward_cond2 = makeIneqUpper(ef_f2,  zeros(size(ef_f2)), 'EF_forward_cond2');
report.ineq.EF_reverse_cond1 = makeIneqUpper(ef_r1,  zeros(size(ef_r1)), 'EF_reverse_cond1');
report.ineq.EF_reverse_cond2 = makeIneqUpper(ef_r2,  zeros(size(ef_r2)), 'EF_reverse_cond2');

%========================
% 7) 具身盒子顶点关系
%========================
AX_re = zeros(Nfe-1,1); AY_re = zeros(Nfe-1,1);
BX_re = zeros(Nfe-1,1); BY_re = zeros(Nfe-1,1);
CX_re = zeros(Nfe-1,1); CY_re = zeros(Nfe-1,1);
DX_re = zeros(Nfe-1,1); DY_re = zeros(Nfe-1,1);

for i = 1:Nfe-1
    ct = cos(theta(i));
    st = sin(theta(i));

    AX_re(i) = x(i) + (LF + up(i))   * ct - (hlb + left(i))  * st;
    BX_re(i) = x(i) + (LF + up(i))   * ct + (hlb + right(i)) * st;
    CX_re(i) = x(i) - (lr + down(i)) * ct + (hlb + right(i)) * st;
    DX_re(i) = x(i) - (lr + down(i)) * ct - (hlb + left(i))  * st;

    AY_re(i) = y(i) + (LF + up(i))   * st + (hlb + left(i))  * ct;
    BY_re(i) = y(i) + (LF + up(i))   * st - (hlb + right(i)) * ct;
    CY_re(i) = y(i) - (lr + down(i)) * st - (hlb + right(i)) * ct;
    DY_re(i) = y(i) - (lr + down(i)) * st + (hlb + left(i))  * ct;
end

report.eq.RELATIONSHIP_AX = makeEqReport(AX - AX_re, 'RELATIONSHIP_AX');
report.eq.RELATIONSHIP_AY = makeEqReport(AY - AY_re, 'RELATIONSHIP_AY');
report.eq.RELATIONSHIP_BX = makeEqReport(BX - BX_re, 'RELATIONSHIP_BX');
report.eq.RELATIONSHIP_BY = makeEqReport(BY - BY_re, 'RELATIONSHIP_BY');
report.eq.RELATIONSHIP_CX = makeEqReport(CX - CX_re, 'RELATIONSHIP_CX');
report.eq.RELATIONSHIP_CY = makeEqReport(CY - CY_re, 'RELATIONSHIP_CY');
report.eq.RELATIONSHIP_DX = makeEqReport(DX - DX_re, 'RELATIONSHIP_DX');
report.eq.RELATIONSHIP_DY = makeEqReport(DY - DY_re, 'RELATIONSHIP_DY');

%========================
% 8) 角点边界约束
%========================
report.ineq.Bounds_AX = makeRangeReport(AX, xmin_box, xmax_box, 'Bounds_AX');
report.ineq.Bounds_BX = makeRangeReport(BX, xmin_box, xmax_box, 'Bounds_BX');
report.ineq.Bounds_CX = makeRangeReport(CX, xmin_box, xmax_box, 'Bounds_CX');
report.ineq.Bounds_DX = makeRangeReport(DX, xmin_box, xmax_box, 'Bounds_DX');

report.ineq.Bounds_AY = makeRangeReport(AY, ymin_box, ymax_box, 'Bounds_AY');
report.ineq.Bounds_BY = makeRangeReport(BY, ymin_box, ymax_box, 'Bounds_BY');
report.ineq.Bounds_CY = makeRangeReport(CY, ymin_box, ymax_box, 'Bounds_CY');
report.ineq.Bounds_DY = makeRangeReport(DY, ymin_box, ymax_box, 'Bounds_DY');

%========================
% 9) 三角面积避障约束
% 严格按当前 WriteEFInitialGuess + NLP.mod：
% - 障碍物只允许三角形或四边形
% - 三角形补成 P1,P2,P3,P3
% - Area = min(rawArea+0.02, rawArea*1.02)
%========================
ppp_vio = [];
ppp_meta = [];   % [i, nn, jj]

A_vio = [];
A_meta = [];     % [i, nn]

B_vio = [];
B_meta = [];

C_vio = [];
C_meta = [];

D_vio = [];
D_meta = [];

interval_min_slack = inf(Nfe-1,1);
interval_worst_obs = zeros(Nfe-1,1);

for i = 2:Nfe-1
    rectArea = (LF + lr + up(i) + down(i)) * (lb + right(i) + left(i));
    worstSlack = inf;
    worstObs = 0;

    for nn = 1:Nobs
        [ox4, oy4] = getPseudoQuad(obs(nn));
        area_nn = getAreaForAMPL(obs(nn));

        % --- eq_PPPoutsideABCD ---
        for jj = 1:4
            px = ox4(jj);
            py = oy4(jj);

            sumTri = triArea([AX(i),AY(i)], [BX(i),BY(i)], [px,py]) + ...
                     triArea([BX(i),BY(i)], [CX(i),CY(i)], [px,py]) + ...
                     triArea([CX(i),CY(i)], [DX(i),DY(i)], [px,py]) + ...
                     triArea([DX(i),DY(i)], [AX(i),AY(i)], [px,py]);

            rhs = rectArea + 0.1;
            vio = max(rhs - sumTri, 0);

            ppp_vio(end+1,1) = vio; %#ok<AGROW>
            ppp_meta(end+1,:) = [i, nn, jj]; %#ok<AGROW>

            slack = sumTri - rhs;
            if slack < worstSlack
                worstSlack = slack;
                worstObs = nn;
            end
        end

        % --- eq_AoutsidePRECTANGLEPPP ---
        sumA = triArea([ox4(1),oy4(1)], [ox4(2),oy4(2)], [AX(i),AY(i)]) + ...
               triArea([ox4(2),oy4(2)], [ox4(3),oy4(3)], [AX(i),AY(i)]) + ...
               triArea([ox4(3),oy4(3)], [ox4(4),oy4(4)], [AX(i),AY(i)]) + ...
               triArea([ox4(4),oy4(4)], [ox4(1),oy4(1)], [AX(i),AY(i)]);

        sumB = triArea([ox4(1),oy4(1)], [ox4(2),oy4(2)], [BX(i),BY(i)]) + ...
               triArea([ox4(2),oy4(2)], [ox4(3),oy4(3)], [BX(i),BY(i)]) + ...
               triArea([ox4(3),oy4(3)], [ox4(4),oy4(4)], [BX(i),BY(i)]) + ...
               triArea([ox4(4),oy4(4)], [ox4(1),oy4(1)], [BX(i),BY(i)]);

        sumC = triArea([ox4(1),oy4(1)], [ox4(2),oy4(2)], [CX(i),CY(i)]) + ...
               triArea([ox4(2),oy4(2)], [ox4(3),oy4(3)], [CX(i),CY(i)]) + ...
               triArea([ox4(3),oy4(3)], [ox4(4),oy4(4)], [CX(i),CY(i)]) + ...
               triArea([ox4(4),oy4(4)], [ox4(1),oy4(1)], [CX(i),CY(i)]);

        sumD = triArea([ox4(1),oy4(1)], [ox4(2),oy4(2)], [DX(i),DY(i)]) + ...
               triArea([ox4(2),oy4(2)], [ox4(3),oy4(3)], [DX(i),DY(i)]) + ...
               triArea([ox4(3),oy4(3)], [ox4(4),oy4(4)], [DX(i),DY(i)]) + ...
               triArea([ox4(4),oy4(4)], [ox4(1),oy4(1)], [DX(i),DY(i)]);

        vioA = max(area_nn - sumA, 0);
        vioB = max(area_nn - sumB, 0);
        vioC = max(area_nn - sumC, 0);
        vioD = max(area_nn - sumD, 0);

        A_vio(end+1,1) = vioA; A_meta(end+1,:) = [i, nn]; %#ok<AGROW>
        B_vio(end+1,1) = vioB; B_meta(end+1,:) = [i, nn]; %#ok<AGROW>
        C_vio(end+1,1) = vioC; C_meta(end+1,:) = [i, nn]; %#ok<AGROW>
        D_vio(end+1,1) = vioD; D_meta(end+1,:) = [i, nn]; %#ok<AGROW>

        cornerSlack = min([sumA - area_nn, sumB - area_nn, sumC - area_nn, sumD - area_nn]);
        if cornerSlack < worstSlack
            worstSlack = cornerSlack;
            worstObs = nn;
        end
    end

    interval_min_slack(i) = worstSlack;
    interval_worst_obs(i) = worstObs;
end

report.ineq.eq_PPPoutsideABCD = packIneq('eq_PPPoutsideABCD', ppp_vio, ppp_meta, {'i','obs','corner'});
report.ineq.eq_AoutsidePRECTANGLEPPP = packIneq('eq_AoutsidePRECTANGLEPPP', A_vio, A_meta, {'i','obs'});
report.ineq.eq_BoutsidePRECTANGLEPPP = packIneq('eq_BoutsidePRECTANGLEPPP', B_vio, B_meta, {'i','obs'});
report.ineq.eq_CoutsidePRECTANGLEPPP = packIneq('eq_CoutsidePRECTANGLEPPP', C_vio, C_meta, {'i','obs'});
report.ineq.eq_DoutsidePRECTANGLEPPP = packIneq('eq_DoutsidePRECTANGLEPPP', D_vio, D_meta, {'i','obs'});

report.plotdata.interval_min_slack = interval_min_slack;
report.plotdata.interval_worst_obs = interval_worst_obs;

%========================
% 10) IPOPT 收敛难度判别指标（经验型）
%========================
report.conv = struct();

% ---------- A. 避障裕度类 ----------
slack_all = report.plotdata.interval_min_slack(:);
if numel(slack_all) >= 3
    slack_use = slack_all(2:end-1);   % 与避障约束 i = 2..Nfe-1 对齐
else
    slack_use = slack_all;
end

report.conv.min_collision_slack  = min(slack_use);
report.conv.mean_collision_slack = mean(slack_use);
report.conv.std_collision_slack  = std(slack_use);

near_thr = 0.05;   % 你可自行调，例如 0.02 / 0.05 / 0.1
report.conv.near_collision_threshold = near_thr;
report.conv.num_near_collision = nnz(slack_use < near_thr);
report.conv.ratio_near_collision = nnz(slack_use < near_thr) / max(numel(slack_use),1);

flag_near = (slack_use < near_thr);
dflag = diff([0; flag_near(:); 0]);
st_run = find(dflag == 1);
ed_run = find(dflag == -1) - 1;
if isempty(st_run)
    report.conv.max_near_collision_run = 0;
else
    report.conv.max_near_collision_run = max(ed_run - st_run + 1);
end

% ---------- B. 换向点局部指标 ----------
sgn_v = sign(v);
for ii = 1:numel(sgn_v)
    if abs(sgn_v(ii)) < 1e-12
        sgn_v(ii) = 0;
    end
end
switch_idx = find(sgn_v(1:end-1) .* sgn_v(2:end) < 0);
report.conv.direction_switch_idx = switch_idx(:);

win = 5;   % 换向点局部窗口半宽，可自行改
switch_local = struct([]);
for kk = 1:numel(switch_idx)
    i0 = switch_idx(kk);
    L  = max(1, i0-win);
    R  = min(Nfe-1, i0+win);

    tmp = struct();
    tmp.idx = i0;
    tmp.window = [L, R];
    tmp.max_dv_res   = max(abs(report.eq.DIFF_dvdt.residual(L:R)));
    tmp.max_dphy_res = max(abs(report.eq.DIFF_dphydt.residual(L:R)));
    tmp.max_dth_res  = max(abs(report.eq.DIFF_dthetadt.residual(L:R)));
    tmp.max_dx_res   = max(abs(report.eq.DIFF_dxdt.residual(L:R)));
    tmp.max_dy_res   = max(abs(report.eq.DIFF_dydt.residual(L:R)));
    tmp.min_dt       = min(dt(L:R));
    tmp.max_abs_w    = max(abs(w(L:R)));
    tmp.max_abs_phy  = max(abs(phy(L:min(R+1,Nfe))));
    tmp.max_abs_v    = max(abs(v(L:min(R+1,Nfe))));
    tmp.min_slack    = min(report.plotdata.interval_min_slack(L:R));
    switch_local = [switch_local; tmp]; %#ok<AGROW>
end
report.conv.switch_local = switch_local;

if isempty(switch_local)
    report.conv.max_switch_dv_res   = 0;
    report.conv.max_switch_dphy_res = 0;
    report.conv.max_switch_dth_res  = 0;
    report.conv.min_switch_dt       = inf;
    report.conv.min_switch_slack    = inf;
else
    report.conv.max_switch_dv_res   = max([switch_local.max_dv_res]);
    report.conv.max_switch_dphy_res = max([switch_local.max_dphy_res]);
    report.conv.max_switch_dth_res  = max([switch_local.max_dth_res]);
    report.conv.min_switch_dt       = min([switch_local.min_dt]);
    report.conv.min_switch_slack    = min([switch_local.min_slack]);
end

% ---------- C. 控制量正向积分漂移 ----------
% 用"最终写入的 v, phy, dt"从初始点正向积分，比较与当前 x,y,theta 的偏离
x_roll  = zeros(Nfe,1);
y_roll  = zeros(Nfe,1);
th_roll = zeros(Nfe,1);

x_roll(1)  = x(1);
y_roll(1)  = y(1);
th_roll(1) = theta(1);

for i = 1:Nfe-1
    x_roll(i+1)  = x_roll(i)  + v(i) * dt(i) * cos(th_roll(i));
    y_roll(i+1)  = y_roll(i)  + v(i) * dt(i) * sin(th_roll(i));
    th_roll(i+1) = th_roll(i) + dt(i) * v(i) * tan(phy(i)) / lw;
end

drift_xy = hypot(x(:) - x_roll(:), y(:) - y_roll(:));
drift_th = abs(wrapToPiLocal(theta(:) - th_roll(:)));

report.conv.rollout = struct();
report.conv.rollout.x_roll = x_roll;
report.conv.rollout.y_roll = y_roll;
report.conv.rollout.th_roll = th_roll;
report.conv.rollout.max_xy_drift   = max(drift_xy);
report.conv.rollout.mean_xy_drift  = mean(drift_xy);
report.conv.rollout.end_xy_drift   = drift_xy(end);
report.conv.rollout.max_th_drift   = max(drift_th);
report.conv.rollout.mean_th_drift  = mean(drift_th);
report.conv.rollout.end_th_drift   = drift_th(end);

% ---------- D. 控制平顺性 / 抖动指标 ----------
dv_node   = diff(v);
dphy_node = diff(phy);
da_node   = diff(a);
dw_node   = diff(w);

report.conv.smoothness = struct();
report.conv.smoothness.tv_v   = sum(abs(dv_node));
report.conv.smoothness.tv_phy = sum(abs(dphy_node));
report.conv.smoothness.tv_a   = sum(abs(da_node));
report.conv.smoothness.tv_w   = sum(abs(dw_node));

report.conv.smoothness.max_dv   = max(abs(dv_node));
report.conv.smoothness.max_dphy = max(abs(dphy_node));
report.conv.smoothness.max_da   = max(abs(da_node));
report.conv.smoothness.max_dw   = max(abs(dw_node));

report.conv.smoothness.rms_dv   = sqrt(mean(dv_node.^2));
report.conv.smoothness.rms_dphy = sqrt(mean(dphy_node.^2));
report.conv.smoothness.rms_da   = sqrt(mean(da_node.^2));
report.conv.smoothness.rms_dw   = sqrt(mean(dw_node.^2));

% ---------- E. 饱和活跃度 ----------
sat_eps = 1e-6;
report.conv.saturation = struct();
report.conv.saturation.ratio_v_sat   = nnz(abs(abs(v)   - v_max)   < sat_eps) / max(numel(v),1);
report.conv.saturation.ratio_phy_sat = nnz(abs(abs(phy) - phy_max) < sat_eps) / max(numel(phy),1);
report.conv.saturation.ratio_a_sat   = nnz(abs(abs(a)   - a_max)   < sat_eps) / max(numel(a),1);
report.conv.saturation.ratio_w_sat   = nnz(abs(abs(w)   - w_max)   < sat_eps) / max(numel(w),1);

% ---------- F. 时间分布刚性 ----------
report.conv.time = struct();
report.conv.time.min_dt = min(dt);
report.conv.time.max_dt = max(dt);
report.conv.time.mean_dt = mean(dt);
report.conv.time.std_dt = std(dt);
report.conv.time.ratio_small_dt = nnz(dt < (min_dt + 0.1*(max_dt-min_dt))) / max(numel(dt),1);
report.conv.time.ratio_large_dt = nnz(dt > (max_dt - 0.1*(max_dt-min_dt))) / max(numel(dt),1);

% ---------- G. 终点邻域误差 ----------
report.conv.terminal = struct();
report.conv.terminal.pos_err = hypot(x(end)-meta.xf, y(end)-meta.yf);
report.conv.terminal.th_err  = abs(wrapToPiLocal(theta(end)-meta.thetaf));

% ---------- H. 经验综合评分（仅供排序参考，不是理论量） ----------
% 分数越大，通常说明 IPOPT 修起来越困难
score = 0;
score = score + 20 * max(0, -report.conv.min_collision_slack);              % 真撞障碍最重
score = score +  8 * report.conv.ratio_near_collision;                      % 大范围贴障
score = score +  5 * min(report.conv.rollout.max_xy_drift, 10);             % 状态漂移
score = score +  3 * min(report.conv.rollout.max_th_drift, 10);             % 航向漂移
score = score +  5 * min(report.conv.max_switch_dphy_res, 10);              % 换向处转向链条不一致
score = score +  3 * min(report.conv.max_switch_dth_res, 10);               % 换向处姿态链条不一致
score = score +  2 * min(report.conv.smoothness.rms_dw, 10);                % 转向抖动
score = score +  1 * min(report.conv.smoothness.rms_da, 10);                % 加速度抖动
score = score +  2 * report.conv.saturation.ratio_phy_sat;                  % 转角卡边界
score = score +  2 * report.conv.saturation.ratio_w_sat;                    % 转角速度卡边界
score = score +  2 * max(0, report.ineq.time_bound2.max_violation);         % tf 超界
score = score +  2 * max(0, report.ineq.time_bound1.max_violation * 100);   % dt 过小/过大
report.conv.score = score;


%========================
% 汇总
%========================
[worstEqName, worstEqVal] = findWorstEq(report.eq);
[worstIneqName, worstIneqVal] = findWorstIneq(report.ineq);

report.summary = struct();
report.summary.worst_equal_constraint = worstEqName;
report.summary.worst_equal_residual = worstEqVal;
report.summary.worst_ineq_constraint = worstIneqName;
report.summary.worst_ineq_violation = worstIneqVal;

%========================
% 终端输出
%========================
fprintf('\n================ Initial Guess Quality Report ================\n');
fprintf('matfile                    : %s\n', matfile);
fprintf('Nfe                        : %d\n', Nfe);
fprintf('Nobs                       : %d\n', Nobs);
fprintf('tol                        : %.3e\n', tol);
fprintf('--------------------------------------------------------------\n');
fprintf('Worst equality residual    : %-32s %.6e\n', worstEqName, worstEqVal);
fprintf('Worst inequality violation : %-32s %.6e\n', worstIneqName, worstIneqVal);
fprintf('--------------------------------------------------------------\n');

printEq(report.eq.define_tf);
printEq(report.eq.DIFF_dxdt);
printEq(report.eq.DIFF_dydt);
printEq(report.eq.DIFF_dvdt);
printEq(report.eq.DIFF_dthetadt);
printEq(report.eq.DIFF_dphydt);

printEq(report.eq.define_kappa);
printEq(report.eq.define_s);
printEq(report.eq.define_splus);
printEq(report.eq.define_sminus);
printEq(report.eq.define_up);
printEq(report.eq.define_down);
printEq(report.eq.define_left);
printEq(report.eq.define_right);

printEq(report.eq.RELATIONSHIP_AX);
printEq(report.eq.RELATIONSHIP_AY);
printEq(report.eq.RELATIONSHIP_BX);
printEq(report.eq.RELATIONSHIP_BY);
printEq(report.eq.RELATIONSHIP_CX);
printEq(report.eq.RELATIONSHIP_CY);
printEq(report.eq.RELATIONSHIP_DX);
printEq(report.eq.RELATIONSHIP_DY);

printIneq(report.ineq.time_bound1);
printIneq(report.ineq.time_bound2);
printIneq(report.ineq.Bonds_v);
printIneq(report.ineq.Bonds_a);
printIneq(report.ineq.Bonds_phy);
printIneq(report.ineq.Bonds_w);

printIneq(report.ineq.EF_arc_bound);
printIneq(report.ineq.EF_forward_cond1);
printIneq(report.ineq.EF_forward_cond2);
printIneq(report.ineq.EF_reverse_cond1);
printIneq(report.ineq.EF_reverse_cond2);

printIneq(report.ineq.Bounds_AX);
printIneq(report.ineq.Bounds_BX);
printIneq(report.ineq.Bounds_CX);
printIneq(report.ineq.Bounds_DX);
printIneq(report.ineq.Bounds_AY);
printIneq(report.ineq.Bounds_BY);
printIneq(report.ineq.Bounds_CY);
printIneq(report.ineq.Bounds_DY);

printIneq(report.ineq.eq_PPPoutsideABCD);
printIneq(report.ineq.eq_AoutsidePRECTANGLEPPP);
printIneq(report.ineq.eq_BoutsidePRECTANGLEPPP);
printIneq(report.ineq.eq_CoutsidePRECTANGLEPPP);
printIneq(report.ineq.eq_DoutsidePRECTANGLEPPP);

fprintf('---------------- IPOPT-oriented difficulty indicators ----------------\n');
fprintf('min collision slack             : %.6e\n', report.conv.min_collision_slack);
fprintf('mean collision slack            : %.6e\n', report.conv.mean_collision_slack);
fprintf('near-collision ratio            : %.6f\n', report.conv.ratio_near_collision);
fprintf('max near-collision run          : %d\n',   report.conv.max_near_collision_run);

fprintf('num direction switches          : %d\n', numel(report.conv.direction_switch_idx));
fprintf('max switch DIFF_dvdt            : %.6e\n', report.conv.max_switch_dv_res);
fprintf('max switch DIFF_dphydt          : %.6e\n', report.conv.max_switch_dphy_res);
fprintf('max switch DIFF_dthetadt        : %.6e\n', report.conv.max_switch_dth_res);
fprintf('min switch dt                   : %.6e\n', report.conv.min_switch_dt);
fprintf('min switch slack                : %.6e\n', report.conv.min_switch_slack);

fprintf('rollout max xy drift            : %.6e\n', report.conv.rollout.max_xy_drift);
fprintf('rollout mean xy drift           : %.6e\n', report.conv.rollout.mean_xy_drift);
fprintf('rollout end xy drift            : %.6e\n', report.conv.rollout.end_xy_drift);
fprintf('rollout max theta drift         : %.6e\n', report.conv.rollout.max_th_drift);

fprintf('TV(v) / TV(phi)                 : %.6e / %.6e\n', ...
    report.conv.smoothness.tv_v, report.conv.smoothness.tv_phy);
fprintf('RMS(da) / RMS(dw)               : %.6e / %.6e\n', ...
    report.conv.smoothness.rms_da, report.conv.smoothness.rms_dw);

fprintf('ratio v/phy/a/w saturation      : %.4f / %.4f / %.4f / %.4f\n', ...
    report.conv.saturation.ratio_v_sat, ...
    report.conv.saturation.ratio_phy_sat, ...
    report.conv.saturation.ratio_a_sat, ...
    report.conv.saturation.ratio_w_sat);

fprintf('terminal pos/theta error        : %.6e / %.6e\n', ...
    report.conv.terminal.pos_err, report.conv.terminal.th_err);

fprintf('difficulty score                : %.6f\n', report.conv.score);

fprintf('==============================================================\n\n');

%========================
% 可视化
%========================
if doPlot
    PlotInitialGuessOverview_Current(report, D, xmin_box, xmax_box, ymin_box, ymax_box);
    %PlotCheck_TimeAndState_Current(D);
    PlotCheck_Dynamics_Current(report, D);
    %PlotCheck_EF_Current(report, D);
    %PlotCheck_Box_Current(report);
    PlotCheck_Collision_Current(report, D, xmin_box, xmax_box, ymin_box, ymax_box);
    %PlotCheck_ConvergenceDifficulty(report, D);
end

%========================
% 内部函数
%========================
    function appendBC(val, name)
        bc(end+1,1) = val; 
        bc_name{end+1} = name; 
    end
end

%======================================================================
% 辅助函数
%======================================================================
function [ox4, oy4] = getPseudoQuad(cur_obs)
ox = cur_obs.x(:);
oy = cur_obs.y(:);

if numel(ox) >= 2
    if abs(ox(1) - ox(end)) < 1e-12 && abs(oy(1) - oy(end)) < 1e-12
        ox(end) = [];
        oy(end) = [];
    end
end

nv = numel(ox);
if nv == 3
    ox4 = [ox; ox(end)];
    oy4 = [oy; oy(end)];
elseif nv == 4
    ox4 = ox;
    oy4 = oy;
else
    error('发现 %d 边形障碍物，但当前 NLP.mod / WriteEFInitialGuess 仅支持三角形或四边形。', nv);
end
end

function area_amp = getAreaForAMPL(cur_obs)
raw = polygonAreaRaw(cur_obs.x(:), cur_obs.y(:));
area_amp = min([raw + 0.02, raw * 1.02]);
end

function A = polygonAreaRaw(X, Y)
X = X(:); Y = Y(:);
if X(1) ~= X(end) || Y(1) ~= Y(end)
    X = [X; X(1)];
    Y = [Y; Y(1)];
end
A = 0;
for ii = 1:length(X)-1
    A = A + X(ii) * Y(ii+1) - Y(ii) * X(ii+1);
end
A = 0.5 * abs(A);
end

function A = triArea(p1, p2, p3)
A = 0.5 * abs((p1(1)-p3(1))*(p2(2)-p3(2)) - (p1(2)-p3(2))*(p2(1)-p3(1)));
end

function S = packIneq(name, vio, meta, metaNames)
S = struct();
S.name = name;
S.max_violation = maxOrZero(vio);
S.mean_violation = meanOrZero(vio);
S.num_bad = nnz(vio > 0);
S.bad_idx = find(vio > 0);
S.violation = vio(:);
S.meta = meta;
S.meta_names = metaNames;
if isempty(meta)
    S.bad_meta = [];
else
    S.bad_meta = meta(vio > 0, :);
end
end

function v = maxOrZero(x)
if isempty(x), v = 0; else, v = max(x); end
end

function v = meanOrZero(x)
if isempty(x), v = 0; else, v = mean(x); end
end

function [name, val] = findWorstEq(eqStruct)
fns = fieldnames(eqStruct);
name = '';
val = 0;
for i = 1:numel(fns)
    s = eqStruct.(fns{i});
    if isstruct(s) && isfield(s, 'max_abs')
        if s.max_abs > val
            val = s.max_abs;
            name = s.name;
        end
    end
end
end

function [name, val] = findWorstIneq(ineqStruct)
fns = fieldnames(ineqStruct);
name = '';
val = 0;
for i = 1:numel(fns)
    s = ineqStruct.(fns{i});
    if isstruct(s) && isfield(s, 'max_violation')
        if s.max_violation > val
            val = s.max_violation;
            name = s.name;
        end
    end
end
end

function printEq(S)
fprintf('%-32s : max|res| = %.6e, bad = %d\n', S.name, S.max_abs, S.num_bad);
end

function printIneq(S)
fprintf('%-32s : max vio  = %.6e, bad = %d\n', S.name, S.max_violation, S.num_bad);
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end