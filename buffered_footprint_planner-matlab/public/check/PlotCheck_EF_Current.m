function PlotCheck_EF_Current(report, D)

figure('Name', 'EF Constraint Check', 'Color', 'w');

subplot(3,2,1);
plot(report.ineq.EF_arc_bound.violation, 'LineWidth', 1.2); grid on;
title('EF\_arc\_bound violation'); xlabel('interval i');

subplot(3,2,2);
plot(report.ineq.EF_forward_cond1.violation, 'LineWidth', 1.2); grid on;
title('EF\_forward\_cond1 violation'); xlabel('interval i');

subplot(3,2,3);
plot(report.ineq.EF_forward_cond2.violation, 'LineWidth', 1.2); grid on;
title('EF\_forward\_cond2 violation'); xlabel('interval i');

subplot(3,2,4);
plot(report.ineq.EF_reverse_cond1.violation, 'LineWidth', 1.2); grid on;
title('EF\_reverse\_cond1 violation'); xlabel('interval i');

subplot(3,2,5);
plot(report.ineq.EF_reverse_cond2.violation, 'LineWidth', 1.2); grid on;
title('EF\_reverse\_cond2 violation'); xlabel('interval i');

subplot(3,2,6);
plot(abs(D.s(:)), 'LineWidth', 1.2); grid on;
title('|s|'); xlabel('interval i');

sgtitle('具身三约束检查');
end