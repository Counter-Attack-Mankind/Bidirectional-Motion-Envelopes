function path_seg = SimulateForward_test(cur_node, v, phy, simu_dt)
global params
Nfe = 20;
x = zeros(1, Nfe);
y = zeros(1, Nfe);
theta = zeros(1, Nfe);
x(1) = cur_node.x;
y(1) = cur_node.y;
theta(1) = cur_node.theta;
dt = simu_dt / (Nfe - 1);
for ii = 2 : Nfe
    theta(ii) = dt * tan(phy) * v / params.vehicle.lw + theta(ii-1);
    x(ii) = x(ii-1) + dt * cos(theta(ii-1)) * v;
    y(ii) = y(ii-1) + dt * sin(theta(ii-1)) * v;
end
phy = ones(1, Nfe) .* phy;
path_seg.x = x;
path_seg.y = y;
path_seg.theta = theta;
path_seg.phy = phy;
end