function PlotCheck_TimeAndState_Current(D)

figure('Name', 'Time and State Check', 'Color', 'w');

subplot(3,2,1);
plot(D.dt(:), 'LineWidth', 1.2); grid on; title('dt'); xlabel('interval i');

subplot(3,2,2);
plot(D.v(:), 'LineWidth', 1.2); grid on; title('v'); xlabel('node i');

subplot(3,2,3);
plot(D.a(:), 'LineWidth', 1.2); grid on; title('a'); xlabel('node i');

subplot(3,2,4);
plot(D.phy(:), 'LineWidth', 1.2); grid on; title('\phi'); xlabel('node i');

subplot(3,2,5);
plot(D.w(:), 'LineWidth', 1.2); grid on; title('w'); xlabel('node i');

subplot(3,2,6);
plot(D.theta(:), 'LineWidth', 1.2); grid on; title('\theta'); xlabel('node i');

sgtitle('时间与状态变量检查');
end