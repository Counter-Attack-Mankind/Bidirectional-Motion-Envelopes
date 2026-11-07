function [h1, h2, h3, h4, h5, h6] = DrawWheels(x, y, theta, phy)
% DrawWheels: draw 4 wheels + front/rear axle lines.
% x,y: rear axle center
% theta: vehicle heading
% phy: steering angle (front wheels), rad
%
% NOTE:
% - Uses params.vehicle.* to stay consistent with vehicle geometry.
% - Rear wheels aligned with theta; front wheels aligned with theta+phy.

global params

% ---- vehicle geometry (consistent with your model) ----
lf  = params.vehicle.lf;
lr  = params.vehicle.lr;
lw  = params.vehicle.lw;
hlb = params.vehicle.hlb;

% ---- wheel appearance ----
wheel_len = 0.30;     % half-length visual scale (same as your lll)
wheel_w   = 0.12;     % wheel half-width (visual)
use_bias  = false;    % set true to keep your original stda bias
stda      = 0.1745;   % ~10deg bias (ONLY if use_bias=true)

% ---- compute 4 wheel centers (AA,BB front; CC,DD rear) ----
AA = [x - sin(theta) * hlb + lw * cos(theta), y + cos(theta) * hlb + lw * sin(theta)]; % front-left
BB = [x + sin(theta) * hlb + lw * cos(theta), y - cos(theta) * hlb + lw * sin(theta)]; % front-right
CC = [x + sin(theta) * hlb,               y - cos(theta) * hlb];                       % rear-right
DD = [x - sin(theta) * hlb,               y + cos(theta) * hlb];                       % rear-left

% ---- helper: build a small rectangle (wheel) around center with heading ang ----
    function P = wheel_poly(center, ang)
        if use_bias
            ang = ang + stda;
        end
        ct = cos(ang); st = sin(ang);
        % rectangle corners around center:
        % along direction: wheel_len, lateral: wheel_w
        p1 = center + [ wheel_len*ct - wheel_w*st,  wheel_len*st + wheel_w*ct];
        p2 = center + [ wheel_len*ct + wheel_w*st,  wheel_len*st - wheel_w*ct];
        p3 = center + [-wheel_len*ct + wheel_w*st, -wheel_len*st - wheel_w*ct];
        p4 = center + [-wheel_len*ct - wheel_w*st, -wheel_len*st + wheel_w*ct];
        P  = [p1; p2; p3; p4; p1]; % close polygon
    end

% ---- rear wheels (no steering) ----
P = wheel_poly(CC, theta);
h1 = fill(P(:,1), P(:,2), [64 0 0] ./ 255, 'EdgeColor','none');

P = wheel_poly(DD, theta);
h2 = fill(P(:,1), P(:,2), [64 0 0] ./ 255, 'EdgeColor','none');

% ---- front wheels (steered) ----
angle = theta + phy;

P = wheel_poly(AA, angle);
h3 = fill(P(:,1), P(:,2), [64 0 0] ./ 255, 'EdgeColor','none');

P = wheel_poly(BB, angle);
h4 = fill(P(:,1), P(:,2), [64 0 0] ./ 255, 'EdgeColor','none');

% ---- axle lines ----
h5 = plot([AA(1), BB(1)], [AA(2), BB(2)], 'k', 'LineWidth', 1.5);
h6 = plot([CC(1), DD(1)], [CC(2), DD(2)], 'k', 'LineWidth', 1.5);

end
