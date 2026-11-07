function [x_full, y_full, theta_full] = InterpolateGrids(x, y, theta)
x_full = [];
y_full = [];
theta_full = [];
for ii = 2 : length(x)
    Nsp = round(norm([x(ii) - x(ii-1), y(ii) - y(ii-1)]) * 1000);
    temp = linspace(x(ii-1), x(ii), Nsp);
    x_full = [x_full, temp(1,1 : (Nsp - 1))];

    temp = linspace(y(ii-1), y(ii), Nsp);
    y_full = [y_full, temp(1,1 : (Nsp - 1))];

    temp = linspace(theta(ii-1), theta(ii), Nsp);
    theta_full = [theta_full, temp(1,1 : (Nsp - 1))];
end
x_full = [x_full, x(end)];
y_full = [y_full, y(end)];
theta_full = [theta_full, theta(end)];
end
