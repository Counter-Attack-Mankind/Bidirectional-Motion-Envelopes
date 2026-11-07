function EmbodiedSamplingVisualize()

% ============================================================
% EmbodiedSamplingVisualize
%
% 用于验证：
%   ✔ 具身盒子方向是否正确
%   ✔ 前进 / 倒车外扩是否正确
%   ✔ 采样点是否合理
%   ✔ AABB参数 a,b,c,d 是否符合预期
%
% 数据来源：
%   params.ef.x / y / theta / a / b / c / d
% ============================================================

global params

%% ===== 数据读取 =====
x  = params.ef.x(:);
y  = params.ef.y(:);
th = params.ef.theta(:);

% ===== 具身盒子 =====
a  = params.ef.a_box(:);
b  = params.ef.b_box(:);
c  = params.ef.c_box(:);
d  = params.ef.d_box(:);

if isfield(params.ef,'phy') && ~isempty(params.ef.phy)
    phy = params.ef.phy(:);
else
    phy = zeros(size(x));
end

if isfield(params.ef,'v') && ~isempty(params.ef.v)
    v = params.ef.v(:);
else
    v = zeros(size(x));
end

n = min([numel(x),numel(y),numel(th),numel(a),numel(b),numel(c),numel(d)]);
x=x(1:n); y=y(1:n); th=th(1:n);
a=a(1:n); b=b(1:n); c=c(1:n); d=d(1:n);
phy=phy(1:n); v=v(1:n);

if n<1
    warning('No data');
    return
end

%% ===== 车辆参数 =====
lf  = params.vehicle.lf;
lw  = params.vehicle.lw;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ===== 画布 =====
figure; clf
hold on; axis equal; grid on
xlabel('x'); ylabel('y')

axis([params.environment.xmin params.environment.xmax ...
      params.environment.ymin params.environment.ymax])

title('Embodied Sampling Debug Visualization')

%% ===== 障碍物 =====
for ii = 1:params.environment.num_obs
    fill(params.environment.obs(ii).x, ...
         params.environment.obs(ii).y,...
         [0.8 0.8 0.8],'EdgeColor','none');
end

%% ===== 轨迹 =====
plot(x,y,'k--','LineWidth',1);

%% ===== 动画循环 =====
for k = 1:n

    cla
    hold on
    axis equal
    grid on
    
    axis([params.environment.xmin params.environment.xmax ...
          params.environment.ymin params.environment.ymax])

    % 障碍物
    for ii = 1:params.environment.num_obs
        fill(params.environment.obs(ii).x,...
             params.environment.obs(ii).y,...
             [0.8 0.8 0.8],'EdgeColor','none');
    end

    % 轨迹
    plot(x,y,'k--','LineWidth',1);

    %% ===== 车身 =====
    [XB,YB] = body_rect(x(k),y(k),th(k),lf,lw,lr,hlb);

    patch(XB,YB,'k',...
        'FaceAlpha',0.25,...
        'EdgeColor','k',...
        'LineWidth',1.5);

    %% ===== 具身盒子 =====
    [XE,YE] = embodied_rect(x(k),y(k),th(k),a(k),b(k),c(k),d(k));

    patch(XE,YE,[0 162 232]/255,...
        'FaceAlpha',0.2,...
        'EdgeColor','b',...
        'LineWidth',1.5);

    %% ===== 方向箭头 =====
    quiver(x(k),y(k),...
        cos(th(k)),sin(th(k)),...
        1,'r','LineWidth',2);

    %% ===== 速度方向标识 =====
    if v(k) >= 0
        dirTxt = 'Forward';
        col = [0 0.6 0];
    else
        dirTxt = 'Reverse';
        col = [0.8 0 0];
    end

    text(x(k),y(k)+1,...
        sprintf('%s\nv=%.2f',dirTxt,v(k)),...
        'Color',col,...
        'FontSize',12,...
        'FontWeight','bold');

    %% ===== 信息 =====
    title(sprintf( ...
        ['k = %d / %d\n' ...
         'theta = %.2f deg\n' ...
         'a=%.2f  b=%.2f  c=%.2f  d=%.2f'],...
         k,n,th(k)*180/pi,a(k),b(k),c(k),d(k)));

    drawnow
    pause(0.05)

end

end

%% ============================================================
%% 车身矩形
%% ============================================================
function [X,Y] = body_rect(x,y,theta,lf,lw,lr,hlb)

c=cos(theta); s=sin(theta);

AX = x + (lf+lw)*c - hlb*s;
AY = y + (lf+lw)*s + hlb*c;

BX = x + (lf+lw)*c + hlb*s;
BY = y + (lf+lw)*s - hlb*c;

CX = x - lr*c + hlb*s;
CY = y - lr*s - hlb*c;

DX = x - lr*c - hlb*s;
DY = y - lr*s + hlb*c;

X=[AX BX CX DX AX];
Y=[AY BY CY DY AY];

end

%% ============================================================
%% 具身盒子矩形
%% ============================================================
function [X,Y] = embodied_rect(x,y,theta,a,b,c,d)

[AX,AY,BX,BY,CX,CY,DX,DY] = ddd2(x,y,theta,a,b,c,d);

X=[AX BX CX DX AX];
Y=[AY BY CY DY AY];

end