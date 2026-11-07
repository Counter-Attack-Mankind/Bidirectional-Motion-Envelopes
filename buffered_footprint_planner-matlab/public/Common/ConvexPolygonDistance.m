function [distance, point_a, point_b] = ConvexPolygonDistance(poly_a, poly_b)
%CONVEXPOLYGONDISTANCE Euclidean distance between two closed convex polygons.
% POLY_A and POLY_B are N-by-2 arrays without a required repeated endpoint.

poly_a = cleanPolygon(poly_a);
poly_b = cleanPolygon(poly_b);

if polygonsIntersect(poly_a, poly_b)
    distance = 0;
    point_a = mean(poly_a, 1);
    point_b = mean(poly_b, 1);
    return;
end

distance = inf;
point_a = [NaN, NaN];
point_b = [NaN, NaN];

for i = 1:size(poly_a, 1)
    a = poly_a(i, :);
    for j = 1:size(poly_b, 1)
        b1 = poly_b(j, :);
        b2 = poly_b(mod(j, size(poly_b, 1)) + 1, :);
        q = closestPointOnSegment(a, b1, b2);
        d = norm(a - q);
        if d < distance
            distance = d;
            point_a = a;
            point_b = q;
        end
    end
end

for i = 1:size(poly_b, 1)
    b = poly_b(i, :);
    for j = 1:size(poly_a, 1)
        a1 = poly_a(j, :);
        a2 = poly_a(mod(j, size(poly_a, 1)) + 1, :);
        q = closestPointOnSegment(b, a1, a2);
        d = norm(b - q);
        if d < distance
            distance = d;
            point_a = q;
            point_b = b;
        end
    end
end
end

function poly = cleanPolygon(poly)
poly = double(poly);
if size(poly, 2) ~= 2 || size(poly, 1) < 3
    error('A polygon must contain at least three 2-D vertices.');
end
if norm(poly(1, :) - poly(end, :)) <= 1e-12
    poly(end, :) = [];
end
end

function tf = polygonsIntersect(a, b)
tf = false;
for i = 1:size(a, 1)
    a1 = a(i, :);
    a2 = a(mod(i, size(a, 1)) + 1, :);
    for j = 1:size(b, 1)
        b1 = b(j, :);
        b2 = b(mod(j, size(b, 1)) + 1, :);
        if segmentsIntersect(a1, a2, b1, b2)
            tf = true;
            return;
        end
    end
end

[in_a, on_a] = inpolygon(a(1,1), a(1,2), b(:,1), b(:,2));
[in_b, on_b] = inpolygon(b(1,1), b(1,2), a(:,1), a(:,2));
tf = in_a || on_a || in_b || on_b;
end

function tf = segmentsIntersect(p1, p2, q1, q2)
tol = 1e-12;
o1 = cross2(p2-p1, q1-p1);
o2 = cross2(p2-p1, q2-p1);
o3 = cross2(q2-q1, p1-q1);
o4 = cross2(q2-q1, p2-q1);

tf = ((o1 > tol && o2 < -tol) || (o1 < -tol && o2 > tol)) && ...
     ((o3 > tol && o4 < -tol) || (o3 < -tol && o4 > tol));
if tf
    return;
end
tf = (abs(o1) <= tol && onSegment(p1, p2, q1, tol)) || ...
     (abs(o2) <= tol && onSegment(p1, p2, q2, tol)) || ...
     (abs(o3) <= tol && onSegment(q1, q2, p1, tol)) || ...
     (abs(o4) <= tol && onSegment(q1, q2, p2, tol));
end

function tf = onSegment(a, b, p, tol)
tf = all(p >= min(a,b)-tol) && all(p <= max(a,b)+tol);
end

function value = cross2(a, b)
value = a(1)*b(2) - a(2)*b(1);
end

function q = closestPointOnSegment(p, a, b)
d = b-a;
den = dot(d,d);
if den <= eps
    q = a;
    return;
end
t = max(0, min(1, dot(p-a,d)/den));
q = a + t*d;
end
