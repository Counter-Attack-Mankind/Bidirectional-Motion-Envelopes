function is_invalid = IsEnlargedBoxInvalid(AX, AY, BX, BY, CX, CY, DX, DY)
global params
is_invalid = 1;

ef_poly = [AX AY;
           BX BY;
           CX CY;
           DX DY];

if any(ef_poly(:,1) > params.environment.xmax) || ...
   any(ef_poly(:,1) < params.environment.xmin) || ...
   any(ef_poly(:,2) > params.environment.ymax) || ...
   any(ef_poly(:,2) < params.environment.ymin)
    return;
end

ef_area_rhs = polygonArea(ef_poly) + 0.1;

for obs_idx = 1 : params.environment.num_obs
    obs_poly = [params.environment.obs(obs_idx).x(:), params.environment.obs(obs_idx).y(:)];
    obs_area_rhs = min(polygonArea(obs_poly) + 0.02, polygonArea(obs_poly) * 1.02);

    % NLP1 obstacle-vertex constraints:
    % each obstacle vertex must stay outside the embodied-footprint box.
    for jj = 1:size(obs_poly, 1)
        area_sum = sumPointToPolygonTriangleAreas(obs_poly(jj, :), ef_poly);
        if area_sum < ef_area_rhs
            return;
        end
    end

    % NLP1 EF-vertex constraints:
    % each embodied-footprint box vertex must stay outside the obstacle.
    for jj = 1:size(ef_poly, 1)
        area_sum = sumPointToPolygonTriangleAreas(ef_poly(jj, :), obs_poly);
        if area_sum < obs_area_rhs
            return;
        end
    end
end

is_invalid = 0;
end

function area_sum = sumPointToPolygonTriangleAreas(P, poly)
area_sum = 0;
n = size(poly, 1);

for i = 1:n
    p1 = poly(i, :);
    if i < n
        p2 = poly(i + 1, :);
    else
        p2 = poly(1, :);
    end

    area_sum = area_sum + triangleArea(P, p1, p2);
end
end

function A = triangleArea(p1, p2, p3)
A = 0.5 * abs((p2(1) - p1(1)) * (p3(2) - p1(2)) - ...
              (p2(2) - p1(2)) * (p3(1) - p1(1)));
end

function A = polygonArea(poly)
x = poly(:, 1);
y = poly(:, 2);
A = 0.5 * abs(sum(x .* y([2:end, 1]) - y .* x([2:end, 1])));
end
