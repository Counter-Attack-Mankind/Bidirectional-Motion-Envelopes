function PlotCheck_ConvergenceDifficulty(report, D)

x = D.x(:); y = D.y(:);
v = D.v(:); phy = D.phy(:); a = D.a(:); w = D.w(:);
dt = D.dt(:);

figure('Name', 'IPOPT Difficulty Indicators', 'Color', 'w');

subplot(3,2,1);
plot(report.plotdata.interval_min_slack, 'LineWidth', 1.2); grid on;
xlabel('interval i'); ylabel('slack');
title('collision slack');

subplot(3,2,2);
plot(report.conv.rollout.x_roll, report.conv.rollout.y_roll, '--', 'LineWidth', 1.0); hold on;
plot(x, y, '-', 'LineWidth', 1.2); grid on; axis equal;
legend('rollout by (v,\phi,dt)', 'current state', 'Location', 'best');
title(sprintf('rollout drift, max xy = %.3e', report.conv.rollout.max_xy_drift));

subplot(3,2,3);
plot(abs(report.eq.DIFF_dphydt.residual), 'LineWidth', 1.2); hold on;
plot(abs(report.eq.DIFF_dthetadt.residual), 'LineWidth', 1.2);
grid on; xlabel('interval i');
legend('|DIFF d\phi|','|DIFF d\theta|','Location','best');
title('turning-chain inconsistency');

subplot(3,2,4);
plot(dt, 'LineWidth', 1.2); hold on;
yline(report.meta.min_dt, '--');
yline(report.meta.max_dt, '--');
grid on; xlabel('interval i');
title('dt distribution');

subplot(3,2,5);
plot(v, 'LineWidth', 1.2); hold on;
plot(phy, 'LineWidth', 1.2);
grid on; xlabel('node i');
legend('v','phy','Location','best');
title('state/control profile');

subplot(3,2,6);
bar([ ...
    report.conv.ratio_near_collision, ...
    report.conv.saturation.ratio_phy_sat, ...
    report.conv.saturation.ratio_w_sat, ...
    report.conv.time.ratio_small_dt ]);
set(gca, 'XTickLabel', {'nearObs','phySat','wSat','smallDt'});
grid on;
title(sprintf('difficulty score = %.3f', report.conv.score));

sgtitle('IPOPT-oriented Convergence Difficulty Indicators');

end