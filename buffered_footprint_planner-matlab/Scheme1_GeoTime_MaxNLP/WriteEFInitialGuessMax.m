function WriteEFInitialGuessMax(x, y, theta, v, a, phy, w, time)
global params

Nfe = params.nfe;

%========================
% 维度整理
%========================
x     = x(:);
y     = y(:);
theta = theta(:);
v     = v(:);
a     = a(:);
phy   = phy(:);
w     = w(:);
time  = time(:);   % 必须是 Nfe-1

assert(length(x)     == Nfe,   'x length must be Nfe');
assert(length(y)     == Nfe,   'y length must be Nfe');
assert(length(theta) == Nfe,   'theta length must be Nfe');
assert(length(v)     == Nfe,   'v length must be Nfe');
assert(length(a)     == Nfe,   'a length must be Nfe');
assert(length(phy)   == Nfe,   'phy length must be Nfe');
assert(length(w)     == Nfe,   'w length must be Nfe');
assert(length(time)  == Nfe-1, 'time length must be Nfe-1');

%========================
% 参数
%========================
lw    = params.vehicle.lw;
LF    = params.vehicle.LF;
lr    = params.vehicle.lr;
hlb   = params.vehicle.hlb;

a_max = params.vehicle.a_max;
w_max = params.vehicle.w_max;

%========================
% 只对区间变量分配 Nfe-1
%========================
dt     = time;                  % Nfe-1
kappa  = tan(phy(1:Nfe-1)) / lw;
kappa_abs = sqrt(kappa.^2 + 1e-8);

s      = zeros(Nfe-1,1);
splus  = zeros(Nfe-1,1);
sminus = zeros(Nfe-1,1);

up     = zeros(Nfe-1,1);
down   = zeros(Nfe-1,1);
left   = zeros(Nfe-1,1);
right  = zeros(Nfe-1,1);

AX = zeros(Nfe-1,1); AY = zeros(Nfe-1,1);
BX = zeros(Nfe-1,1); BY = zeros(Nfe-1,1);
CX = zeros(Nfe-1,1); CY = zeros(Nfe-1,1);
DX = zeros(Nfe-1,1); DY = zeros(Nfe-1,1);

raw_a = zeros(Nfe-1,1);
raw_w = zeros(Nfe-1,1);

%========================
% 工程型初值：由 v, phy, dt 差分得到 a, w
% 再进行限幅
% 注意：这里只更新 1..Nfe-1，末端再单独处理
%========================
for ii = 1:(Nfe-1)
    if dt(ii) > 0
        raw_w(ii) = (phy(ii+1) - phy(ii)) / dt(ii);
        raw_a(ii) = (v(ii+1)   - v(ii))   / dt(ii);

        w(ii) = min(max(raw_w(ii), -w_max), w_max);
        a(ii) = min(max(raw_a(ii), -a_max), a_max);
    else
        raw_w(ii) = 0;
        raw_a(ii) = 0;
        w(ii) = 0;
        a(ii) = 0;
    end
end

%========================
% 首末端边界（与 NLP.mod 一致）
%========================
w(1)   = 0;
a(1)   = 0;
w(Nfe) = 0;
a(Nfe) = 0;

v(1)     = 0;
phy(1)   = 0;
v(Nfe)   = 0;
phy(Nfe) = 0;

% 首末端修正后，重新计算区间曲率
kappa = tan(phy(1:Nfe-1)) / lw;
kappa_abs = sqrt(kappa.^2 + 1e-8);

%========================
% 区间量：1..Nfe-1
%========================
for ii = 1:(Nfe-1)
    s(ii)      = v(ii) * dt(ii);
    splus(ii)  = max(s(ii), 0);
    sminus(ii) = max(-s(ii), 0);

    kk = kappa(ii);
    sp = splus(ii);
    sm = sminus(ii);

    up(ii)   = sp + hlb * kappa_abs(ii) * sp;
    down(ii) = sm + hlb * kappa_abs(ii) * sm;

    left(ii) = max(-lr * kk * sp, (LF + 0.5 * sp) * kk * sp) ...
             + max(-LF * kk * sm, (lr + 0.5 * sm) * kk * sm);

    right(ii)= max( lr * kk * sp, -(LF + 0.5 * sp) * kk * sp) ...
             + max( LF * kk * sm, -(lr + 0.5 * sm) * kk * sm);
end

%========================
% 盒子顶点：也是 1..Nfe-1
%========================
for ii = 1:(Nfe-1)
    ct = cos(theta(ii));
    st = sin(theta(ii));

    AX(ii) = x(ii) + (LF + up(ii)) * ct - (hlb + left(ii))  * st;
    BX(ii) = x(ii) + (LF + up(ii)) * ct + (hlb + right(ii)) * st;
    CX(ii) = x(ii) - (lr + down(ii)) * ct + (hlb + right(ii)) * st;
    DX(ii) = x(ii) - (lr + down(ii)) * ct - (hlb + left(ii))  * st;

    AY(ii) = y(ii) + (LF + up(ii)) * st + (hlb + left(ii))  * ct;
    BY(ii) = y(ii) + (LF + up(ii)) * st - (hlb + right(ii)) * ct;
    CY(ii) = y(ii) - (lr + down(ii)) * st - (hlb + right(ii)) * ct;
    DY(ii) = y(ii) - (lr + down(ii)) * st + (hlb + left(ii))  * ct;
end

%========================
% 简单诊断打印
%========================
fprintf('\n=========== Initial Guess Diagnostics ===========\n');
fprintf('Nfe                     : %d\n', Nfe);
fprintf('sum(time)               : %.6f\n', sum(time));
fprintf('min/max dt              : %.6e / %.6e\n', min(time), max(time));
fprintf('max |raw_a|             : %.6e\n', max(abs(raw_a)));
fprintf('max |raw_w|             : %.6e\n', max(abs(raw_w)));
fprintf('a clipped ratio         : %.6f\n', mean(abs(raw_a) > a_max));
fprintf('w clipped ratio         : %.6f\n', mean(abs(raw_w) > w_max));
fprintf('=================================================\n\n');

%========================
% 写 ig.INIVAL
%========================
if exist('ig.INIVAL','file')
    delete('ig.INIVAL');
end
fid = fopen('ig.INIVAL', 'w');

% ---- 节点变量：1..Nfe ----
for ii = 1:Nfe
    fprintf(fid, 'let x[%d] := %.12f;\r\n',     ii, x(ii));
    fprintf(fid, 'let y[%d] := %.12f;\r\n',     ii, y(ii));
    fprintf(fid, 'let theta[%d] := %.12f;\r\n', ii, theta(ii));
    fprintf(fid, 'let v[%d] := %.12f;\r\n',     ii, v(ii));
    fprintf(fid, 'let a[%d] := %.12f;\r\n',     ii, a(ii));
    fprintf(fid, 'let phy[%d] := %.12f;\r\n',   ii, phy(ii));
    fprintf(fid, 'let w[%d] := %.12f;\r\n',     ii, w(ii));
end

% ---- 区间变量：1..Nfe-1 ----
for ii = 1:(Nfe-1)
    fprintf(fid, 'let k[%d] := %.12f;\r\n',      ii, kappa(ii));
    fprintf(fid, 'let dt[%d] := %.12f;\r\n',     ii, dt(ii));
    fprintf(fid, 'let s[%d] := %.12f;\r\n',      ii, s(ii));
    fprintf(fid, 'let splus[%d] := %.12f;\r\n',  ii, splus(ii));
    fprintf(fid, 'let sminus[%d] := %.12f;\r\n', ii, sminus(ii));

    fprintf(fid, 'let up[%d] := %.12f;\r\n',     ii, up(ii));
    fprintf(fid, 'let down[%d] := %.12f;\r\n',   ii, down(ii));
    fprintf(fid, 'let left[%d] := %.12f;\r\n',   ii, left(ii));
    fprintf(fid, 'let right[%d] := %.12f;\r\n',  ii, right(ii));

    fprintf(fid, 'let AX[%d] := %.12f;\r\n', ii, AX(ii));
    fprintf(fid, 'let AY[%d] := %.12f;\r\n', ii, AY(ii));
    fprintf(fid, 'let BX[%d] := %.12f;\r\n', ii, BX(ii));
    fprintf(fid, 'let BY[%d] := %.12f;\r\n', ii, BY(ii));
    fprintf(fid, 'let CX[%d] := %.12f;\r\n', ii, CX(ii));
    fprintf(fid, 'let CY[%d] := %.12f;\r\n', ii, CY(ii));
    fprintf(fid, 'let DX[%d] := %.12f;\r\n', ii, DX(ii));
    fprintf(fid, 'let DY[%d] := %.12f;\r\n', ii, DY(ii));
end

fprintf(fid, 'let tf := %.12f;\r\n', sum(time));
fclose(fid);

params.ef.ig.time = time;
params.task.thetaf = theta(end);

%========================
% 写 PV
%========================
if exist('PV','file')
    delete('PV');
end
fid = fopen('PV', 'w');
fprintf(fid, '1  %.12f\r\n', params.task.x0);
fprintf(fid, '2  %.12f\r\n', params.task.y0);
fprintf(fid, '3  %.12f\r\n', params.task.theta0);
fprintf(fid, '4  %.12f\r\n', params.task.xf);
fprintf(fid, '5  %.12f\r\n', params.task.yf);
fprintf(fid, '6  %.12f\r\n', params.task.thetaf);
fprintf(fid, '7  %d\r\n',    params.nfe);
fprintf(fid, '8  %.12f\r\n', params.vehicle.v_max);
fprintf(fid, '9  %.12f\r\n', params.vehicle.phy_max);
fprintf(fid, '10 %.12f\r\n', params.vehicle.a_max);
fprintf(fid, '11 %.12f\r\n', params.vehicle.w_max);
fprintf(fid, '12 %.12f\r\n', params.vehicle.lw);
fprintf(fid, '13 %.12f\r\n', params.vehicle.LF);
fprintf(fid, '14 %.12f\r\n', params.vehicle.lr);
fprintf(fid, '15 %.12f\r\n', params.vehicle.hlb);
fprintf(fid, '16 %d\r\n',    params.environment.num_obs);
fprintf(fid, '17 %.12f\r\n', params.ef.max_dt);
fprintf(fid, '18 %.12f\r\n', params.ef.min_dt);
fclose(fid);

%========================
% 写 PPP，障碍物的坐标顶点
% 自动兼容三角形：补成伪四边形 P1,P2,P3,P3
%========================
if exist('PPP','file')
    delete('PPP');
end
fid = fopen('PPP', 'w');
for ii = 1:params.environment.num_obs
    cur_obs = params.environment.obs(ii);
    ox = cur_obs.x(:);
    oy = cur_obs.y(:);
    % ---------- 去掉首尾重复闭合点 ----------
    if numel(ox) >= 2
        if abs(ox(1) - ox(end)) < 1e-12 && abs(oy(1) - oy(end)) < 1e-12
            ox(end) = [];
            oy(end) = [];
        end
    end
    nv = numel(ox);
    % ---------- 只允许三角形或四边形 ----------
    if nv == 3            % 三角形补成伪四边形：P1,P2,P3,P3
        ox = [ox; ox(end)];
        oy = [oy; oy(end)];
    elseif nv == 4        % 四边形直接保留
    else
        fclose(fid);
        error('第 %d 个障碍物有 %d 个顶点，仅支持三角形或四边形。', ii, nv);
    end
    % ---------- 写入 PPP ----------
    for jj = 1:4
        fprintf(fid, '%d %d %d %.12f\r\n', ii, jj, 1, ox(jj));
        fprintf(fid, '%d %d %d %.12f\r\n', ii, jj, 2, oy(jj));
    end
end
fclose(fid);

%========================
% 写 Area
%========================
if exist('Area','file')
    delete('Area');
end
fid = fopen('Area', 'w');
for ii = 1:params.environment.num_obs
    area = CalculatePolygonArea(params.environment.obs(ii));
    fprintf(fid, '%d %.12f\r\n', ii, min([area + 0.02, area * 1.02]));
end
fclose(fid);

%========================
% 保存检查数据
%========================
data = struct();

data.x     = x;
data.y     = y;
data.theta = theta;
data.v     = v;
data.a     = a;
data.phy   = phy;
data.w     = w;

data.dt     = dt;
data.tf     = sum(dt);

data.kappa  = kappa;
data.kabs   = kappa_abs;
data.s      = s;
data.splus  = splus;
data.sminus = sminus;

data.up     = up;
data.down   = down;
data.left   = left;
data.right  = right;

data.AX = AX; data.AY = AY;
data.BX = BX; data.BY = BY;
data.CX = CX; data.CY = CY;
data.DX = DX; data.DY = DY;

data.meta = struct();
data.meta.Nfe      = params.nfe;
data.meta.x0       = params.task.x0;
data.meta.y0       = params.task.y0;
data.meta.theta0   = params.task.theta0;
data.meta.xf       = params.task.xf;
data.meta.yf       = params.task.yf;
data.meta.thetaf   = params.task.thetaf;

data.meta.v_max    = params.vehicle.v_max;
data.meta.phy_max  = params.vehicle.phy_max;
data.meta.a_max    = params.vehicle.a_max;
data.meta.w_max    = params.vehicle.w_max;

data.meta.lw       = params.vehicle.lw;
data.meta.LF       = params.vehicle.LF;
data.meta.lr       = params.vehicle.lr;
data.meta.hlb      = params.vehicle.hlb;
data.meta.max_dt   = params.ef.max_dt;
data.meta.min_dt   = params.ef.min_dt;

data.meta.Nobs     = params.environment.num_obs;
data.meta.obs      = params.environment.obs;
data.meta.note     = 'Initial guess aligned with NLP1.mod';

save('written_initial_guess_data.mat', 'data');

% Use exactly the same OBCA data and dual initialization as Schemes 1/2.
PrepareOBCAData('written_initial_guess_data.mat', 'ig.INIVAL', ...
    'OBCAData.dat', 0.01);

end

function area = CalculatePolygonArea(V)
X = V.x(:);
Y = V.y(:);

if X(1) ~= X(end) || Y(1) ~= Y(end)
    X = [X; X(1)];
    Y = [Y; Y(1)];
end

area = 0;
for ii = 1:length(X)-1
    area = area + X(ii) * Y(ii+1) - Y(ii) * X(ii+1);
end
area = 0.5 * abs(area);
end
