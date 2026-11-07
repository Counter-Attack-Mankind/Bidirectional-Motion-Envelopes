%% (远处仍然走大步，靠近障碍物才切小步)
function local_dt = GetAdaptiveSimuDuration(x, y)
global params

dist_min = inf;

for ii = 1:params.environment.num_obs
    ox = params.environment.obs(ii).x(:);
    oy = params.environment.obs(ii).y(:);

    % 先用障碍物顶点做一个粗距离判定
    dist_min = min(dist_min, min(hypot(ox - x, oy - y)));
end

% ===== 两档模式，可先这样试 =====
if dist_min < 3.0
    local_dt = 2;   % 近障碍：小步长
else
    local_dt = params.ha.simu_unit_duration;   % 远障碍：保持全局默认
end
end