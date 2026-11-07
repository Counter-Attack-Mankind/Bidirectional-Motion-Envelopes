function PrepareOBCAData(matfile, igfile, datafile, dmin)
%PREPAREOBCADATA Write normalized obstacle halfspaces and OBCA warm starts.
% The obstacle convention is ObsA(n,e,:)*z <= Obsb(n,e).

if nargin < 1 || isempty(matfile), matfile = 'written_initial_guess_data.mat'; end
if nargin < 2 || isempty(igfile), igfile = 'ig.INIVAL'; end
if nargin < 3 || isempty(datafile), datafile = 'OBCAData.dat'; end
if nargin < 4 || isempty(dmin), dmin = 0.01; end
if ~(isscalar(dmin) && isfinite(dmin) && dmin > 0)
    error('OBCA dmin must be finite and strictly positive.');
end

S = load(matfile);
if ~isfield(S, 'data'), error('No variable named data found in %s.', matfile); end
D = S.data;

[nedge, obs_a, obs_b, polygons] = obstacleHalfspaces(D.meta.obs);
nbox = numel(D.AX);
nobs = numel(nedge);

lambda = zeros(nbox, nobs, 4);
mu = zeros(nbox, nobs, 4);
qx = zeros(nbox, nobs);
qy = zeros(nbox, nobs);
distance = nan(nbox, nobs);
certificate = nan(nbox, nobs);

for i = 2:nbox
    body = [D.AX(i), D.AY(i); D.BX(i), D.BY(i); ...
            D.CX(i), D.CY(i); D.DX(i), D.DY(i)];
    for n = 1:nobs
        [dist, pbody, pobs] = ConvexPolygonDistance(body, polygons{n});
        q = pbody-pobs;
        if norm(q) <= 1e-10
            q = mean(body,1)-mean(polygons{n},1);
        end
        if norm(q) <= 1e-10, q = obs_a(n,1,:); q = q(:).'; end
        q = q/norm(q);

        lam = representDirection(obs_a(n,1:nedge(n),:), ...
            obs_b(n,1:nedge(n)), pobs, q);
        r = [cos(D.theta(i))*q(1)+sin(D.theta(i))*q(2), ...
            -sin(D.theta(i))*q(1)+cos(D.theta(i))*q(2)];
        muv = [max(-r(1),0), max(r(1),0), max(-r(2),0), max(r(2),0)];

        % OBCA only requires ||q|| <= 1. Starting every pair at ||q||=1
        % makes thousands of norm constraints active simultaneously and is
        % numerically degenerate. Scale well-separated pairs into the cone
        % interior while retaining a valid distance certificate.
        if dist > 0
            rho = min(1,max(0.25,(dmin+1e-3)/dist));
            q = rho*q;
            lam = rho*lam;
            muv = rho*muv;
        end

        lambda(i,n,1:nedge(n)) = lam;
        mu(i,n,:) = muv;
        qx(i,n) = q(1);
        qy(i,n) = q(2);
        distance(i,n) = dist;

        g = bodyLocalBounds(D, i);
        ax = reshape(obs_a(n,1:nedge(n),1),[],1);
        ay = reshape(obs_a(n,1:nedge(n),2),[],1);
        bb = reshape(obs_b(n,1:nedge(n)),[],1);
        at_minus_b = ax*D.x(i) + ay*D.y(i) - bb;
        certificate(i,n) = at_minus_b(:).'*lam(:) - g(:).'*muv(:);
    end
end

D.meta.obca_dmin = dmin;
D.obca = struct('Nedge', nedge, 'A', obs_a, 'b', obs_b, ...
    'lambda', lambda, 'mu', mu, 'qx', qx, 'qy', qy, ...
    'distance', distance, 'certificate', certificate);
data = D; %#ok<NASGU>
save(matfile, 'data');

writeOBCAData(datafile, dmin, nedge, obs_a, obs_b);
appendWarmStart(igfile, lambda, mu, qx, qy, nedge);

valid = certificate(2:end,:);
fprintf('OBCA warm start: minimum certificate %.6e m, required %.6e m.\n', ...
    min(valid(:)), dmin);
end

function g = bodyLocalBounds(D, i)
if isfield(D, 'up')
    g = [D.meta.LF+D.up(i), D.meta.lr+D.down(i), ...
         D.meta.hlb+D.left(i), D.meta.hlb+D.right(i)];
else
    g = [D.meta.LF, D.meta.lr, D.meta.hlb, D.meta.hlb];
end
end

function [nedge, all_a, all_b, polygons] = obstacleHalfspaces(obstacles)
nobs = numel(obstacles);
nedge = zeros(nobs,1);
all_a = zeros(nobs,4,2);
all_b = zeros(nobs,4);
polygons = cell(nobs,1);
for n = 1:nobs
    p = [obstacles(n).x(:), obstacles(n).y(:)];
    if size(p,1) >= 2 && norm(p(1,:)-p(end,:)) <= 1e-12, p(end,:) = []; end
    nv = size(p,1);
    if nv < 3 || nv > 4
        error('OBCA currently supports convex triangles/quadrilaterals; obstacle %d has %d vertices.', n, nv);
    end
    assertConvex(p, n);
    c = mean(p,1);
    for e = 1:nv
        p1 = p(e,:); p2 = p(mod(e,nv)+1,:);
        edge = p2-p1;
        normal = [edge(2), -edge(1)];
        normal = normal/norm(normal);
        bound = dot(normal,p1);
        if dot(normal,c) > bound
            normal = -normal;
            bound = -bound;
        end
        all_a(n,e,:) = normal;
        all_b(n,e) = bound;
    end
    nedge(n) = nv;
    polygons{n} = p;
end
end

function assertConvex(p, obstacle_index)
turns = zeros(size(p,1),1);
for i = 1:size(p,1)
    a = p(mod(i,size(p,1))+1,:)-p(i,:);
    b = p(mod(i+1,size(p,1))+1,:)-p(mod(i,size(p,1))+1,:);
    turns(i) = a(1)*b(2)-a(2)*b(1);
end
turns = turns(abs(turns)>1e-10);
if isempty(turns) || (any(turns>0) && any(turns<0))
    error('Obstacle %d is degenerate or non-convex; convex decomposition is required.', obstacle_index);
end
end

function lambda = representDirection(a3, b, boundary_point, q)
a = reshape(a3, size(a3,2), 2);
m = size(a,1);
active = find(abs(a*boundary_point(:)-b(:)) <= 1e-6);
best_score = inf;
best_residual = inf;
lambda = zeros(m,1);
for i = 1:m
    candidate = max(0, dot(a(i,:),q))*double((1:m)'==i);
    residual = norm(a.'*candidate-q);
    support_error = abs(b(:).'*candidate-dot(q,boundary_point));
    score = support_error + 1e3*residual;
    if score < best_score, best_score=score; best_residual=residual; lambda=candidate; end
end
for i = 1:m
    for j = i+1:m
        pair = [a(i,:).', a(j,:).'];
        if abs(det(pair)) <= 1e-12, continue; end
        coeff = pair\q(:);
        if all(coeff >= -1e-10)
            candidate = zeros(m,1);
            candidate([i,j]) = max(coeff,0);
            residual = norm(a.'*candidate-q(:));
            support_error = abs(b(:).'*candidate-dot(q,boundary_point));
            score = support_error + 1e3*residual;
            if score < best_score, best_score=score; best_residual=residual; lambda=candidate; end
        end
    end
end
if best_residual > 1e-7
    if isempty(active)
        [~,idx] = max(a*q(:));
    else
        [~,local_idx] = max(a(active,:)*q(:));
        idx = active(local_idx);
    end
    lambda=zeros(m,1); lambda(idx)=1;
end
end

function writeOBCAData(filename, dmin, nedge, a, b)
fid = fopen(filename,'w');
if fid < 0, error('Cannot create %s.', filename); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'param dmin := %.12f;\r\n', dmin);
fprintf(fid, 'param Nedge :=\r\n');
for n=1:numel(nedge), fprintf(fid,'%d %d\r\n',n,nedge(n)); end
fprintf(fid, ';\r\nparam ObsA :=\r\n');
for n=1:numel(nedge)
    for e=1:4
        for d=1:2, fprintf(fid,'%d %d %d %.12f\r\n',n,e,d,a(n,e,d)); end
    end
end
fprintf(fid, ';\r\nparam Obsb :=\r\n');
for n=1:numel(nedge)
    for e=1:4, fprintf(fid,'%d %d %.12f\r\n',n,e,b(n,e)); end
end
fprintf(fid, ';\r\n');
end

function appendWarmStart(filename, lambda, mu, qx, qy, nedge)
fid = fopen(filename,'a');
if fid < 0, error('Cannot append to %s.', filename); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
for i=2:size(lambda,1)
    for n=1:numel(nedge)
        fprintf(fid,'let obca_qx[%d,%d] := %.12f;\r\n',i,n,qx(i,n));
        fprintf(fid,'let obca_qy[%d,%d] := %.12f;\r\n',i,n,qy(i,n));
        for e=1:4
            fprintf(fid,'let obca_lambda[%d,%d,%d] := %.12f;\r\n',i,n,e,lambda(i,n,e));
            fprintf(fid,'let obca_mu[%d,%d,%d] := %.12f;\r\n',i,n,e,mu(i,n,e));
        end
    end
end
end
