function [AX, AY, BX, BY, CX, CY, DX, DY] = ddd2(x, y, theta, a, b, c, d)
global params
lf = params.vehicle.lf;
lr = params.vehicle.lr;
lw = params.vehicle.lw;
hlb = params.vehicle.hlb;

AX = x + (lf + lw + a) * cos(theta) - (hlb + b) * sin(theta);
AY = y + (lf + lw + a) * sin(theta) + (hlb + b) * cos(theta);
BX = x + (lf + lw + a) * cos(theta) + (hlb + d) * sin(theta);
BY = y + (lf + lw + a) * sin(theta) - (hlb + d) * cos(theta);
CX = x - (lr + c) * cos(theta) + (hlb + d) * sin(theta);
CY = y - (lr + c) * sin(theta) - (hlb + d) * cos(theta);
DX = x - (lr + c) * cos(theta) - (hlb + b) * sin(theta);
DY = y - (lr + c) * sin(theta) + (hlb + b) * cos(theta);
end