function [AX, AY, BX, BY, CX, CY, DX, DY] = ddd(x, y, theta)
global params
lf = params.vehicle.lf;
lr = params.vehicle.lr;
lw = params.vehicle.lw;
hlb = params.vehicle.hlb;

AX = x + (lf + lw) * cos(theta) - hlb * sin(theta);
AY = y + (lf + lw) * sin(theta) + hlb * cos(theta);
BX = x + (lf + lw) * cos(theta) + hlb * sin(theta);
BY = y + (lf + lw) * sin(theta) - hlb * cos(theta);
CX = x - lr * cos(theta) + hlb * sin(theta);
CY = y - lr * sin(theta) - hlb * cos(theta);
DX = x - lr * cos(theta) - hlb * sin(theta);
DY = y - lr * sin(theta) + hlb * cos(theta);
end