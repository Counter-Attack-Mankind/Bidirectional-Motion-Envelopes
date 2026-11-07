function flag = LoadEFOptimumAndRefine(strategy_dir)
global params

base_path = fullfile(strategy_dir, 'runtime');

if ~isfolder(base_path)
    error('Runtime optimized variable folder does not exist: %s', base_path);
end

flag_file = fullfile(base_path, 'opti_flag.txt');
if exist(flag_file, 'file') == 2
    opti_flag = load(flag_file);
else
    warning('opti_flag.txt does not exist: %s', flag_file);
    opti_flag = 0;
end

x = readVec(base_path, 'x.txt');
y = readVec(base_path, 'y.txt');
theta = readVec(base_path, 'theta.txt');

v = readVec(base_path, 'v.txt');
a = readVec(base_path, 'a.txt');
phy = readVec(base_path, 'phy.txt');
w = readVec(base_path, 'w.txt');

dt = readVec(base_path, 'dt.txt');
s = readVec(base_path, 's.txt');
k = readVec(base_path, 'k.txt');

up = readVec(base_path, 'up.txt');
down = readVec(base_path, 'down.txt');
left = readVec(base_path, 'left.txt');
right = readVec(base_path, 'right.txt');

x = x(:); y = y(:); theta = theta(:);
v = v(:); a = a(:); phy = phy(:); w = w(:);
dt = dt(:); s = s(:); k = k(:);
up = up(:); down = down(:); left = left(:); right = right(:);

Nstate = length(x);
Nseg = Nstate - 1;
assert(length(dt) == Nseg, 'dt length must be Nstate-1.');
assert(length(s) == Nseg, 's length must be Nstate-1.');
assert(length(k) == Nseg, 'k length must be Nstate-1.');

theta_unwrap = theta;
for i = 2:Nstate
    while theta_unwrap(i) - theta_unwrap(i-1) > pi
        theta_unwrap(i) = theta_unwrap(i) - 2*pi;
    end
    while theta_unwrap(i) - theta_unwrap(i-1) < -pi
        theta_unwrap(i) = theta_unwrap(i) + 2*pi;
    end
end

params.ef.x = x;
params.ef.y = y;
params.ef.theta = theta_unwrap;
params.ef.v = v;
params.ef.a = a;
params.ef.phy = phy;
params.ef.w = w;
params.ef.dt = dt;
params.ef.s = s;
params.ef.k = k;
params.ef.up = up;
params.ef.down = down;
params.ef.left = left;
params.ef.right = right;
params.ef.tf = sum(dt);

populateEnrichedEf();

flag = logical(opti_flag);
end

function vec = readVec(base_path, name)
file = fullfile(base_path, name);
if exist(file, 'file') ~= 2
    error('Missing optimized variable file: %s', file);
end
vec = load(file);
vec = vec(:);
end

function populateEnrichedEf()
global params

x = params.ef.x;
y = params.ef.y;
theta = params.ef.theta;
v = params.ef.v;
a = params.ef.a;
phy = params.ef.phy;
w = params.ef.w;
dt = params.ef.dt;
up = params.ef.up;
down = params.ef.down;
left = params.ef.left;
right = params.ef.right;

Nseg = numel(dt);
base_step = 1e-4;

x_enriched = [];
y_enriched = [];
theta_enriched = [];
v_enriched = [];
a_enriched = [];
phy_enriched = [];
w_enriched = [];
up_enriched = [];
down_enriched = [];
left_enriched = [];
right_enriched = [];

for ii = 1:Nseg
    nfe_seg = max(2, round(dt(ii) / base_step) + 1);

    x_seg = linspace(x(ii), x(ii+1), nfe_seg);
    y_seg = linspace(y(ii), y(ii+1), nfe_seg);
    th_seg = linspace(theta(ii), theta(ii+1), nfe_seg);

    v_seg = linspace(v(ii), v(ii+1), nfe_seg);
    a_seg = linspace(a(ii), a(ii+1), nfe_seg);
    phy_seg = linspace(phy(ii), phy(ii+1), nfe_seg);
    w_seg = linspace(w(ii), w(ii+1), nfe_seg);

    up_seg = repmat(up(ii), 1, nfe_seg);
    down_seg = repmat(down(ii), 1, nfe_seg);
    left_seg = repmat(left(ii), 1, nfe_seg);
    right_seg = repmat(right(ii), 1, nfe_seg);

    if ii > 1
        x_seg(1) = []; y_seg(1) = []; th_seg(1) = [];
        v_seg(1) = []; a_seg(1) = []; phy_seg(1) = []; w_seg(1) = [];
        up_seg(1) = []; down_seg(1) = []; left_seg(1) = []; right_seg(1) = [];
    end

    x_enriched = [x_enriched, x_seg]; %#ok<AGROW>
    y_enriched = [y_enriched, y_seg]; %#ok<AGROW>
    theta_enriched = [theta_enriched, th_seg]; %#ok<AGROW>
    v_enriched = [v_enriched, v_seg]; %#ok<AGROW>
    a_enriched = [a_enriched, a_seg]; %#ok<AGROW>
    phy_enriched = [phy_enriched, phy_seg]; %#ok<AGROW>
    w_enriched = [w_enriched, w_seg]; %#ok<AGROW>
    up_enriched = [up_enriched, up_seg]; %#ok<AGROW>
    down_enriched = [down_enriched, down_seg]; %#ok<AGROW>
    left_enriched = [left_enriched, left_seg]; %#ok<AGROW>
    right_enriched = [right_enriched, right_seg]; %#ok<AGROW>
end

nfe_vis = max(2, round(params.ef.tf * 96));
index = round(linspace(1, length(x_enriched), nfe_vis));
index = unique(index, 'stable');

params.ef.enriched_x = x_enriched(index).';
params.ef.enriched_y = y_enriched(index).';
params.ef.enriched_theta = theta_enriched(index).';
params.ef.enriched_v = v_enriched(index).';
params.ef.enriched_a = a_enriched(index).';
params.ef.enriched_phy = phy_enriched(index).';
params.ef.enriched_w = w_enriched(index).';
params.ef.enriched_up = up_enriched(index).';
params.ef.enriched_down = down_enriched(index).';
params.ef.enriched_left = left_enriched(index).';
params.ef.enriched_right = right_enriched(index).';
end
