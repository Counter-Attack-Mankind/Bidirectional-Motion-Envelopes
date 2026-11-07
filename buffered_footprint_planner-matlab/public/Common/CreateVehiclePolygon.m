function V = CreateVehiclePolygon(x, y, theta)
global params
cos_theta = cos(theta);
sin_theta = sin(theta);

AX = x + params.vehicle.LF * cos_theta - params.vehicle.hlb * sin_theta;
BX = x + params.vehicle.LF * cos_theta + params.vehicle.hlb * sin_theta;
CX = x - params.vehicle.lr * cos_theta + params.vehicle.hlb * sin_theta;
DX = x - params.vehicle.lr * cos_theta - params.vehicle.hlb * sin_theta;
AY = y + params.vehicle.LF * sin_theta + params.vehicle.hlb * cos_theta;
BY = y + params.vehicle.LF * sin_theta - params.vehicle.hlb * cos_theta;
CY = y - params.vehicle.lr * sin_theta - params.vehicle.hlb * cos_theta;
DY = y - params.vehicle.lr * sin_theta + params.vehicle.hlb * cos_theta;
V.x = [AX, BX, CX, DX, AX];
V.y = [AY, BY, CY, DY, AY];
end