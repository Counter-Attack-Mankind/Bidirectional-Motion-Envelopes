function [terminal_time, x0, y0, theta0, v, a] = CalculateTimeStamp(x, y, theta)
% Calculate path length
Nfe = length(x);
path_length = 0;
for ii = 2 : Nfe
    path_length = path_length + hypot(x(ii) - x(ii-1), y(ii) - y(ii-1));
end

% Calculate s(t), a(t) and v(t) locally
[s, v, a, terminal_time] = SolveMinTimeOPtimalControlProblem(path_length);
% Refine x(t), y(t), and theta(t)
[x0, y0, theta0] = InterpolateGrids(x, y, theta);

Nfe = length(x0);
local_s = zeros(1, Nfe);
path_length0 = 0;
for ii = 2 : Nfe
    path_length0 = path_length0 + hypot(x0(ii) - x0(ii-1), y0(ii) - y0(ii-1));
    local_s(ii) = path_length0;
end

ind = zeros(1, size(s,2));
for ii = 1 : length(s)
    temp = abs(s(ii) - local_s);
    temp = find(temp == min(temp));
    ind(ii) = temp(end);
end
x0 = x0(ind);
y0 = y0(ind);
theta0 = theta0(ind);
end