function VisualizeInitialEFCollision(matfile)
%VISUALIZEINITIALEFCOLLISION Show OBCA-clear and violating initial boxes.
global params
if nargin<1 || isempty(matfile), matfile='written_initial_guess_data.mat'; end
S=load(matfile);
if ~isfield(S,'data'), error('No variable named data found in %s.',matfile); end
D=S.data;
nbox=numel(D.AX);
dmin=0.01;
if isfield(D.meta,'obca_dmin'), dmin=D.meta.obca_dmin; end
collision=false(nbox,1);
clearance=inf(nbox,1);
for i=2:nbox
    body=[D.AX(i),D.AY(i);D.BX(i),D.BY(i);D.CX(i),D.CY(i);D.DX(i),D.DY(i)];
    for n=1:D.meta.Nobs
        obs=[D.meta.obs(n).x(:),D.meta.obs(n).y(:)];
        if size(obs,1)>=2 && norm(obs(1,:)-obs(end,:))<=1e-12, obs(end,:)=[]; end
        clearance(i)=min(clearance(i),ConvexPolygonDistance(body,obs));
    end
    collision(i)=clearance(i)<dmin-1e-9;
end

figure('Name','Initial EF OBCA Clearance Check','Color','w');
hold on; axis equal; box on; grid on; xlabel('x / m'); ylabel('y / m');
axis([params.environment.xmin,params.environment.xmax, ...
      params.environment.ymin,params.environment.ymax]);
for n=1:numel(params.environment.obs)
    patch(params.environment.obs(n).x(:),params.environment.obs(n).y(:), ...
        [0.75 0.75 0.75],'EdgeColor',[0.2 0.2 0.2], ...
        'FaceAlpha',0.8,'HandleVisibility','off');
end
plot(D.x,D.y,'k--','LineWidth',1.0,'DisplayName','Initial trajectory');
for i=1:nbox
    X=[D.AX(i),D.BX(i),D.CX(i),D.DX(i),D.AX(i)];
    Y=[D.AY(i),D.BY(i),D.CY(i),D.DY(i),D.AY(i)];
    if collision(i)
        patch(X,Y,[1.0 0.65 0.65],'FaceAlpha',0.35, ...
            'EdgeColor',[0.85 0.05 0.05],'LineWidth',2,'HandleVisibility','off');
        text(mean(X(1:4)),mean(Y(1:4)),sprintf('%d',i), ...
            'Color',[0.85 0.05 0.05],'FontWeight','bold','HorizontalAlignment','center');
    else
        patch(X,Y,[0.70 0.84 1.0],'FaceAlpha',0.10, ...
            'EdgeColor',[0.10 0.35 0.85],'LineWidth',0.8,'HandleVisibility','off');
    end
end
h1=plot(nan,nan,'-','Color',[0.10 0.35 0.85],'LineWidth',1.5,'DisplayName','OBCA-clear EF');
h2=plot(nan,nan,'-','Color',[0.85 0.05 0.05],'LineWidth',2,'DisplayName','OBCA violation');
legend([h1,h2],'Location','bestoutside');
bad=find(collision);
title(sprintf('Initial Buffered Footprints: %d violating / %d checked',numel(bad),max(nbox-1,0)));
fprintf('\n========== Initial EF OBCA Clearance Check ==========\n');
fprintf('checked EF boxes : %d\n',max(nbox-1,0));
fprintf('violating boxes  : %d\n',numel(bad));
fprintf('required distance: %.6f m\n',dmin);
if ~isempty(bad), fprintf('indices          : '); fprintf('%d ',bad); fprintf('\n'); end
fprintf('=====================================================\n\n');
end
