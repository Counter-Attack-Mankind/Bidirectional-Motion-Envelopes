function ShrinkWrittenInitialGuessEF(matfile)
% LEGACY: geometric shrink is not used by the current Scheme1/2 workflow.
%SHRINKWRITTENINITIALGUESSEF Repair the EF warm start against OBCA distance.
% Only the four interval buffer dimensions are reduced. Vehicle states,
% controls, and the time grid are not changed.
global params
if nargin < 1 || isempty(matfile), matfile = 'written_initial_guess_data.mat'; end
S = load(matfile);
if ~isfield(S,'data'), error('No variable named data found in %s.',matfile); end
D = S.data;
scale_min = params.ef.shrink.scale_min;
scale_step = params.ef.shrink.scale_step;
dmin = getObcaDmin(D);
nbox = numel(D.AX);
num_shrunk = 0;
num_unresolved = 0;

% NLP collision constraints apply to intervals 2,...,Nfe-1.
for i = 2:nbox
    if intervalObstacleMinSlack(D,i,dmin) >= 0, continue; end
    num_shrunk = num_shrunk+1;
    repaired = false;
    scale_grid = 1:-scale_step:scale_min;
    if isempty(scale_grid) || abs(scale_grid(end)-scale_min)>1e-12
        scale_grid = [scale_grid,scale_min]; %#ok<AGROW>
    end
    for scale = scale_grid
        C = makeScaledInterval(D,i,scale);
        if intervalObstacleMinSlack(C,i,dmin) >= 0
            D = copyIntervalEF(D,C,i);
            repaired = true;
            break;
        end
    end
    if ~repaired
        D = copyIntervalEF(D,makeScaledInterval(D,i,0),i);
        num_unresolved = num_unresolved+1;
    end
end

data = D; %#ok<NASGU>
save(matfile,'data');
rewriteIgInival(D);
PrepareOBCAData(matfile,'ig.INIVAL','OBCAData.dat',dmin);

fprintf('\n========== OBCA Geometric Feasibility Enhancement ==========\n');
fprintf('shrunk EF boxes      : %d\n',num_shrunk);
fprintf('unresolved at scale0 : %d\n',num_unresolved);
fprintf('required clearance   : %.6f m\n',dmin);
fprintf('==============================================================\n\n');
end

function C = makeScaledInterval(D,i,scale)
C = D;
C.up(i)=scale*D.up(i); C.down(i)=scale*D.down(i);
C.left(i)=scale*D.left(i); C.right(i)=scale*D.right(i);
[C.AX(i),C.AY(i),C.BX(i),C.BY(i),C.CX(i),C.CY(i),C.DX(i),C.DY(i)] = ...
    intervalBoxVertices(C,i);
end

function D = copyIntervalEF(D,C,i)
fields = {'up','down','left','right','AX','AY','BX','BY','CX','CY','DX','DY'};
for k=1:numel(fields), D.(fields{k})(i)=C.(fields{k})(i); end
end

function [AX,AY,BX,BY,CX,CY,DX,DY] = intervalBoxVertices(D,i)
LF=D.meta.LF; lr=D.meta.lr; hlb=D.meta.hlb;
ct=cos(D.theta(i)); st=sin(D.theta(i));
AX=D.x(i)+(LF+D.up(i))*ct-(hlb+D.left(i))*st;
AY=D.y(i)+(LF+D.up(i))*st+(hlb+D.left(i))*ct;
BX=D.x(i)+(LF+D.up(i))*ct+(hlb+D.right(i))*st;
BY=D.y(i)+(LF+D.up(i))*st-(hlb+D.right(i))*ct;
CX=D.x(i)-(lr+D.down(i))*ct+(hlb+D.right(i))*st;
CY=D.y(i)-(lr+D.down(i))*st-(hlb+D.right(i))*ct;
DX=D.x(i)-(lr+D.down(i))*ct-(hlb+D.left(i))*st;
DY=D.y(i)-(lr+D.down(i))*st+(hlb+D.left(i))*ct;
end

function slack = intervalObstacleMinSlack(D,i,dmin)
body=[D.AX(i),D.AY(i);D.BX(i),D.BY(i);D.CX(i),D.CY(i);D.DX(i),D.DY(i)];
slack=inf;
for n=1:D.meta.Nobs
    obs=[D.meta.obs(n).x(:),D.meta.obs(n).y(:)];
    if size(obs,1)>=2 && norm(obs(1,:)-obs(end,:))<=1e-12, obs(end,:)=[]; end
    slack=min(slack,ConvexPolygonDistance(body,obs)-dmin);
end
end

function dmin = getObcaDmin(D)
if isfield(D.meta,'obca_dmin'), dmin=D.meta.obca_dmin; else, dmin=0.01; end
end

function rewriteIgInival(D)
fid=fopen('ig.INIVAL','w');
if fid<0, error('Cannot open ig.INIVAL for writing.'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
Nfe=D.meta.Nfe;
for i=1:Nfe
    fprintf(fid,'let x[%d] := %.12f;\r\n',i,D.x(i));
    fprintf(fid,'let y[%d] := %.12f;\r\n',i,D.y(i));
    fprintf(fid,'let theta[%d] := %.12f;\r\n',i,D.theta(i));
    fprintf(fid,'let v[%d] := %.12f;\r\n',i,D.v(i));
    fprintf(fid,'let a[%d] := %.12f;\r\n',i,D.a(i));
    fprintf(fid,'let phy[%d] := %.12f;\r\n',i,D.phy(i));
    fprintf(fid,'let w[%d] := %.12f;\r\n',i,D.w(i));
end
for i=1:Nfe-1
    fprintf(fid,'let k[%d] := %.12f;\r\n',i,D.kappa(i));
    fprintf(fid,'let dt[%d] := %.12f;\r\n',i,D.dt(i));
    fprintf(fid,'let s[%d] := %.12f;\r\n',i,D.s(i));
    fprintf(fid,'let splus[%d] := %.12f;\r\n',i,D.splus(i));
    fprintf(fid,'let sminus[%d] := %.12f;\r\n',i,D.sminus(i));
    fprintf(fid,'let up[%d] := %.12f;\r\n',i,D.up(i));
    fprintf(fid,'let down[%d] := %.12f;\r\n',i,D.down(i));
    fprintf(fid,'let left[%d] := %.12f;\r\n',i,D.left(i));
    fprintf(fid,'let right[%d] := %.12f;\r\n',i,D.right(i));
    names={'AX','AY','BX','BY','CX','CY','DX','DY'};
    for j=1:numel(names), fprintf(fid,'let %s[%d] := %.12f;\r\n',names{j},i,D.(names{j})(i)); end
end
fprintf(fid,'let tf := %.12f;\r\n',D.tf);
end
