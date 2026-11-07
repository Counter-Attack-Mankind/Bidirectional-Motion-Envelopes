function PlotCheck_Box_Current(report)

figure('Name', 'Box Geometry Check', 'Color', 'w');

subplot(2,2,1);
plot(abs(report.eq.RELATIONSHIP_AX.residual), 'LineWidth', 1.2); hold on;
plot(abs(report.eq.RELATIONSHIP_BX.residual), 'LineWidth', 1.2);
plot(abs(report.eq.RELATIONSHIP_CX.residual), 'LineWidth', 1.2);
plot(abs(report.eq.RELATIONSHIP_DX.residual), 'LineWidth', 1.2);
grid on; title('角点 X 关系残差'); xlabel('interval i');
legend('AX','BX','CX','DX');

subplot(2,2,2);
plot(abs(report.eq.RELATIONSHIP_AY.residual), 'LineWidth', 1.2); hold on;
plot(abs(report.eq.RELATIONSHIP_BY.residual), 'LineWidth', 1.2);
plot(abs(report.eq.RELATIONSHIP_CY.residual), 'LineWidth', 1.2);
plot(abs(report.eq.RELATIONSHIP_DY.residual), 'LineWidth', 1.2);
grid on; title('角点 Y 关系残差'); xlabel('interval i');
legend('AY','BY','CY','DY');

subplot(2,2,3);
hold on; grid on;
plot(report.ineq.Bounds_AX.violation, 'LineWidth', 1.2);
plot(report.ineq.Bounds_BX.violation, 'LineWidth', 1.2);
plot(report.ineq.Bounds_CX.violation, 'LineWidth', 1.2);
plot(report.ineq.Bounds_DX.violation, 'LineWidth', 1.2);
title('角点 X 边界 violation'); xlabel('interval i');
legend('AX','BX','CX','DX');

subplot(2,2,4);
hold on; grid on;
plot(report.ineq.Bounds_AY.violation, 'LineWidth', 1.2);
plot(report.ineq.Bounds_BY.violation, 'LineWidth', 1.2);
plot(report.ineq.Bounds_CY.violation, 'LineWidth', 1.2);
plot(report.ineq.Bounds_DY.violation, 'LineWidth', 1.2);
title('角点 Y 边界 violation'); xlabel('interval i');
legend('AY','BY','CY','DY');

sgtitle('具身盒子几何与边界检查');
end