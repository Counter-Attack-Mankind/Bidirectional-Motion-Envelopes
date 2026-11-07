function [AX, AY, BX, BY, CX, CY, DX, DY] = ddd3(x, y, theta, a, b, c, d)
global params
rate = params.nlp.threshold_rate;
lf = params.vehicle.lf + (1 - rate) * 0.2;
lr = params.vehicle.lr + (1 - rate) * 0.2;
lw = params.vehicle.lw;
hlb = params.vehicle.hlb + (1 - rate) * 0.1;

AX = x + (lf + lw + a) * cos(theta) - (hlb + b) * sin(theta);
AY = y + (lf + lw + a) * sin(theta) + (hlb + b) * cos(theta);
BX = x + (lf + lw + a) * cos(theta) + (hlb + d) * sin(theta);
BY = y + (lf + lw + a) * sin(theta) - (hlb + d) * cos(theta);
CX = x - (lr + c) * cos(theta) + (hlb + d) * sin(theta);
CY = y - (lr + c) * sin(theta) - (hlb + d) * cos(theta);
DX = x - (lr + c) * cos(theta) - (hlb + b) * sin(theta);
DY = y - (lr + c) * sin(theta) + (hlb + b) * cos(theta);
end