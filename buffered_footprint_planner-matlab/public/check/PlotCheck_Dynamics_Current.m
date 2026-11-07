function PlotCheck_Dynamics_Current(report, D)

figure('Name', 'Dynamics Residual Check', 'Color', 'w');

subplot(3,2,1);
stem(abs(report.eq.DIFF_dxdt.residual), 'filled'); grid on;
title('|DIFF\_dxdt|'); xlabel('interval i');

subplot(3,2,2);
stem(abs(report.eq.DIFF_dydt.residual), 'filled'); grid on;
title('|DIFF\_dydt|'); xlabel('interval i');

subplot(3,2,3);
stem(abs(report.eq.DIFF_dvdt.residual), 'filled'); grid on;
title('|DIFF\_dvdt|'); xlabel('interval i');

subplot(3,2,4);
stem(abs(report.eq.DIFF_dthetadt.residual), 'filled'); grid on;
title('|DIFF\_dthetadt|'); xlabel('interval i');

subplot(3,2,5);
stem(abs(report.eq.DIFF_dphydt.residual), 'filled'); grid on;
title('|DIFF\_dphydt|'); xlabel('interval i');

subplot(3,2,6);
plot(D.dt(:), 'LineWidth', 1.2); grid on;
title('dt'); xlabel('interval i');

sgtitle('车辆运动学离散约束残差');
end