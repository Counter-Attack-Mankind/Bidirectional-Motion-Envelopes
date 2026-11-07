function [x, y, theta, v, a, phy, w, time] = RepairInitialGuessEFOnly(x, y, theta, v, a, phy, w, time)

global params

opts = getRepairOptions();

x = x(:);
y = y(:);
theta = unwrapLocal(theta(:));
v = v(:);
a = a(:);
phy = phy(:);
w = w(:);
dt = normalizeDt(time(:), numel(x));

repair_log = struct();
repair_log.iter = [];
repair_log.num_bad = [];
repair_log.min_slack = [];
repair_log.inserted = [];
repair_log.bad_idx = {};

fprintf('\n========== EF Repair ==========\n');
fprintf('max_iter                    : %d\n', opts.max_iter);
fprintf('insert_margin               : %d\n', opts.insert_margin);
fprintf('max_nfe                     : %d\n', opts.max_nfe);

for iter = 1:opts.max_iter
    report = CheckInitialEFCollision(x, y, theta, v, phy, dt, false);

    repair_log.iter(end+1,1) = iter;
    repair_log.num_bad(end+1,1) = report.num_bad;
    repair_log.min_slack(end+1,1) = report.min_slack;
    repair_log.bad_idx{end+1,1} = report.bad_idx;

    fprintf('repair iter %d: Nfe=%d, bad=%d, min_slack=%.6e\n', ...
        iter, numel(x), report.num_bad, report.min_slack);

    if report.num_bad == 0
        fprintf('EF repair converged.\n');
        break;
    end

    split_idx = expandBadIndices(report.bad_idx, numel(x)-1, opts.insert_margin);
    if isempty(split_idx)
        warning('No split interval found although report has bad boxes.');
        break;
    end

    if numel(x) + numel(split_idx) > opts.max_nfe
        allowed = max(opts.max_nfe - numel(x), 0);
        split_idx = split_idx(1:min(allowed, numel(split_idx)));
        if isempty(split_idx)
            warning('EF repair stopped: max_nfe reached before all collisions were repaired.');
            break;
        end
    end

    [x, y, theta, v, a, phy, w, dt] = splitIntervals(x, y, theta, v, a, phy, w, dt, split_idx);
    repair_log.inserted(end+1,1) = numel(split_idx);
end

final_report = CheckInitialEFCollision(x, y, theta, v, phy, dt, opts.show_final_plot);
fprintf('final repair status: Nfe=%d, bad=%d, min_slack=%.6e\n', ...
    numel(x), final_report.num_bad, final_report.min_slack);
fprintf('========================================\n\n');

params.nfe = numel(x);
params.ef.repair_log = repair_log;
params.ef.repair_report = final_report;

time = [dt(:); 0];

% Return row vectors to match the rest of this codebase.
x = x(:).';
y = y(:).';
theta = theta(:).';
v = v(:).';
a = a(:).';
phy = phy(:).';
w = w(:).';
time = time(:).';
end

function opts = getRepairOptions()
global params
opts.max_iter = 6;
opts.insert_margin = 1;
opts.max_nfe = 450;
opts.show_final_plot = true;

if isfield(params.ef, 'repair')
    user_opts = params.ef.repair;
    fields = fieldnames(opts);
    for i = 1:numel(fields)
        f = fields{i};
        if isfield(user_opts, f)
            opts.(f) = user_opts.(f);
        end
    end
end
end

function dt = normalizeDt(time, Nfe)
if numel(time) == Nfe
    dt = time(1:end-1);
elseif numel(time) == Nfe - 1
    dt = time;
else
    error('time length must be Nfe or Nfe-1.');
end
dt = dt(:);
pos_dt = dt(dt > 0 & isfinite(dt));
if isempty(pos_dt)
    fallback_dt = 0.05;
else
    fallback_dt = min(pos_dt);
end
dt(dt <= 0 | ~isfinite(dt)) = fallback_dt;
if any(~isfinite(dt)) || any(dt <= 0)
    dt(:) = 0.05;
end
end

function idx = expandBadIndices(bad_idx, Nseg, margin)
if isempty(bad_idx)
    idx = [];
    return;
end
idx = [];
for k = 1:numel(bad_idx)
    idx = [idx, (bad_idx(k)-margin):(bad_idx(k)+margin)]; %#ok<AGROW>
end
idx = unique(idx);
idx = idx(idx >= 1 & idx <= Nseg);
end

function [x, y, theta, v, a, phy, w, dt] = splitIntervals(x, y, theta, v, a, phy, w, dt, split_idx)
split_idx = unique(split_idx(:).', 'stable');
split_idx = sort(split_idx, 'descend');

for idx = split_idx
    xm = 0.5 * (x(idx) + x(idx+1));
    ym = 0.5 * (y(idx) + y(idx+1));
    thm = 0.5 * (theta(idx) + theta(idx+1));

    vm = midpointVelocity(v(idx), v(idx+1));
    am = 0.5 * (a(idx) + a(idx+1));
    phym = 0.5 * (phy(idx) + phy(idx+1));
    wm = 0.5 * (w(idx) + w(idx+1));

    old_dt = dt(idx);
    dt1 = 0.5 * old_dt;
    dt2 = old_dt - dt1;

    x = insertAfter(x, idx, xm);
    y = insertAfter(y, idx, ym);
    theta = insertAfter(theta, idx, thm);
    v = insertAfter(v, idx, vm);
    a = insertAfter(a, idx, am);
    phy = insertAfter(phy, idx, phym);
    w = insertAfter(w, idx, wm);
    dt = [dt(1:idx-1); dt1; dt2; dt(idx+1:end)];
end

[a, w] = recomputeControls(v, phy, dt);
theta = unwrapLocal(theta);
end

function out = insertAfter(vec, idx, val)
out = [vec(1:idx); val; vec(idx+1:end)];
end

function vm = midpointVelocity(v1, v2)
if sign(v1) ~= sign(v2) && abs(v1) > 1e-8 && abs(v2) > 1e-8
    vm = 0;
else
    vm = 0.5 * (v1 + v2);
end
end

function [a, w] = recomputeControls(v, phy, dt)
global params
Nfe = numel(v);
a = zeros(Nfe,1);
w = zeros(Nfe,1);
for i = 1:Nfe-1
    if dt(i) > 0
        a(i) = (v(i+1) - v(i)) / dt(i);
        w(i) = (phy(i+1) - phy(i)) / dt(i);
    end
end
a = min(max(a, -params.vehicle.a_max), params.vehicle.a_max);
w = min(max(w, -params.vehicle.w_max), params.vehicle.w_max);
a(1) = 0; a(end) = 0;
w(1) = 0; w(end) = 0;
end

function theta = unwrapLocal(theta)
theta = theta(:);
for i = 2:numel(theta)
    while theta(i) - theta(i-1) > pi
        theta(i) = theta(i) - 2*pi;
    end
    while theta(i) - theta(i-1) < -pi
        theta(i) = theta(i) + 2*pi;
    end
end
end
