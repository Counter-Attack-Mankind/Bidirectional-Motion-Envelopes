function PlotCheck_Collision_Current(report, D, xmin_box, xmax_box, ymin_box, ymax_box)

x = D.x(:); y = D.y(:); v = D.v(:);
AX = D.AX(:); AY = D.AY(:);
BX = D.BX(:); BY = D.BY(:);
CX = D.CX(:); CY = D.CY(:);
DX = D.DX(:); DY = D.DY(:);
obs = D.meta.obs;

interval_min_slack = report.plotdata.interval_min_slack(:);

figure('Name', 'Collision Check Visualization', 'Color', 'w');
hold on; axis equal; box on; grid on;
axis([xmin_box, xmax_box, ymin_box, ymax_box]);
xlabel('x'); ylabel('y');
title('三角面积避障检查可视化');

for ii = 1:numel(obs)
    fill(obs(ii).x(:), obs(ii).y(:), [0.82 0.82 0.82], ...
        'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 1.0);
    cx = mean(obs(ii).x(:));
    cy = mean(obs(ii).y(:));
    text(cx, cy, sprintf('%d', ii), 'Color', [0.2 0.2 0.2], ...
        'HorizontalAlignment', 'center', 'FontSize', 10);
end

plot(x, y, 'k--', 'LineWidth', 1.0);

badIdx  = find(interval_min_slack < 0);
warnIdx = find(interval_min_slack >= 0 & interval_min_slack < 0.05);

for i = 1:numel(AX)
    XX = [AX(i) BX(i) CX(i) DX(i) AX(i)];
    YY = [AY(i) BY(i) CY(i) DY(i) AY(i)];

    if ismember(i, badIdx)
        col = [0.85 0.1 0.1];
        lw = 2.0;
    elseif ismember(i, warnIdx)
        col = [0.95 0.6 0.1];
        lw = 1.5;
    else
        if v(i) >= 0
            col = [0.2 0.45 0.9];
        else
            col = [0.75 0.25 0.25];
        end
        lw = 0.8;
    end

    plot(XX, YY, '-', 'Color', col, 'LineWidth', lw);
end

[~, order] = sort(interval_min_slack, 'ascend');
topK = min(10, nnz(isfinite(interval_min_slack)));
topIdx = order(1:topK);

for kk = 1:topK
    i = topIdx(kk);
    if i < 1 || i > numel(AX), continue; end
    cx = mean([AX(i), BX(i), CX(i), DX(i)]);
    cy = mean([AY(i), BY(i), CY(i), DY(i)]);
    text(cx, cy, sprintf('%d', i), 'Color', [0 0 0], ...
        'FontSize', 9, 'HorizontalAlignment', 'center');
end

legend({'障碍物','轨迹','具身盒子'}, 'Location', 'bestoutside');

figure('Name', 'Collision Slack by Interval', 'Color', 'w');
plot(interval_min_slack, 'LineWidth', 1.2); grid on;
xlabel('interval i');
ylabel('min slack');
title('每个区间最小避障裕度（<0 表示违反）');
end