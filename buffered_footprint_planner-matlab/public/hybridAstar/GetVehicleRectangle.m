function poly = GetVehicleRectangle(x, y, theta)
global params

AX = x + params.vehicle.LF * cos(theta) - params.vehicle.hlb * sin(theta);
AY = y + params.vehicle.LF * sin(theta) + params.vehicle.hlb * cos(theta);

BX = x + params.vehicle.LF * cos(theta) + params.vehicle.hlb * sin(theta);
BY = y + params.vehicle.LF * sin(theta) - params.vehicle.hlb * cos(theta);

CX = x - params.vehicle.lr * cos(theta) + params.vehicle.hlb * sin(theta);
CY = y - params.vehicle.lr * sin(theta) - params.vehicle.hlb * cos(theta);

DX = x - params.vehicle.lr * cos(theta) - params.vehicle.hlb * sin(theta);
DY = y - params.vehicle.lr * sin(theta) + params.vehicle.hlb * cos(theta);

poly = [AX AY;
        BX BY;
        CX CY;
        DX DY];
end