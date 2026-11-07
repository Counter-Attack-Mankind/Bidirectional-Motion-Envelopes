function [h_forward, h_reverse] = DrawGradientVehicleSweptArea(ax, x, y, theta, v, vehicle, forward_color, reverse_color)
% Draw vehicle swept area as a pose-by-pose translucent gradient.
% It intentionally avoids a single union/envelope polygon.

x = x(:);
y = y(:);
theta = theta(:);
v = v(:);

n = min([numel(x), numel(y), numel(theta), numel(v)]);
x = x(1:n);
y = y(1:n);
theta = theta(1:n);
v = v(1:n);

lf  = vehicle.lf;
lw  = vehicle.lw;
lr  = vehicle.lr;
hlb = vehicle.hlb;

if nargin < 7 || isempty(forward_color)
    forward_color = [0.12 0.32 0.58];
end
if nargin < 8 || isempty(reverse_color)
    reverse_color = [0.58 0.18 0.16];
end

alpha_min = 0.045;
alpha_max = 0.34;
eps_dir = 1e-10;

for k = 1:n
    [XB, YB] = bodyRectCorners(x(k), y(k), theta(k), lf, lw, lr, hlb);
    tau = (k - 1) / max(n - 1, 1);
    face_alpha = alpha_min + (alpha_max - alpha_min) * tau;

    if v(k) >= -eps_dir
        face_color = forward_color;
    else
        face_color = reverse_color;
    end

    patch(ax, XB, YB, face_color, ...
        'FaceAlpha', face_alpha, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end

h_forward = patch(ax, nan, nan, forward_color, ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', 'none');
h_reverse = patch(ax, nan, nan, reverse_color, ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', 'none');
end

function [X, Y] = bodyRectCorners(x, y, theta, lf, lw, lr, hlb)
c = cos(theta);
s = sin(theta);

AX = x + (lf + lw) * c - hlb * s;
AY = y + (lf + lw) * s + hlb * c;

BX = x + (lf + lw) * c + hlb * s;
BY = y + (lf + lw) * s - hlb * c;

CX = x - lr * c + hlb * s;
CY = y - lr * s - hlb * c;

DX = x - lr * c - hlb * s;
DY = y - lr * s + hlb * c;

X = [AX BX CX DX AX];
Y = [AY BY CY DY AY];
end
