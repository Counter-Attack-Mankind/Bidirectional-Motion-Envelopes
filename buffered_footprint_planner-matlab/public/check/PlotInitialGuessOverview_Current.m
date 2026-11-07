function PlotInitialGuessOverview_Current(report, D, xmin_box, xmax_box, ymin_box, ymax_box)

x = D.x(:); y = D.y(:); v = D.v(:);
AX = D.AX(:); AY = D.AY(:);
BX = D.BX(:); BY = D.BY(:);
CX = D.CX(:); CY = D.CY(:);
DX = D.DX(:); DY = D.DY(:);
obs = D.meta.obs;

figure('Name', 'Initial Guess Overview', 'Color', 'w');
hold on; axis equal; box on; grid on;
axis([xmin_box, xmax_box, ymin_box, ymax_box]);
xlabel('x'); ylabel('y');
title('最终送入 NLP.mod 的初始解轨迹与具身盒子');

for ii = 1:numel(obs)
    fill(obs(ii).x(:), obs(ii).y(:), [0.82 0.82 0.82], ...
        'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 1.0);
end

plot(x, y, 'k-', 'LineWidth', 1.1);

idF = find(v >= 0);
idR = find(v < 0);
if ~isempty(idF)
    plot(x(idF), y(idF), '.', 'MarkerSize', 12, 'Color', [0.12 0.35 0.9]);
end
if ~isempty(idR)
    plot(x(idR), y(idR), '.', 'MarkerSize', 12, 'Color', [0.85 0.2 0.2]);
end

plot(x(1), y(1), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(x(end), y(end), 'mo', 'MarkerSize', 8, 'LineWidth', 1.5);

for i = 1:numel(AX)
    XX = [AX(i) BX(i) CX(i) DX(i) AX(i)];
    YY = [AY(i) BY(i) CY(i) DY(i) AY(i)];
    if v(i) >= 0
        col = [0.2 0.45 0.9];
    else
        col = [0.85 0.25 0.25];
    end
    plot(XX, YY, '-', 'Color', col, 'LineWidth', 0.8);
end

legend({'障碍物','轨迹','前进点','倒车点','起点','终点'}, 'Location', 'bestoutside');
end