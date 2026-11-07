function [a, b, c, d] = EstimateAABBnew(s, k, sgn)
% LEGACY: LSE footprint helper used only by TimeDistribution_old.
% Estimate EF box dimensions with the same LSE smoothing as NLP1.mod.
global params

hlb = params.vehicle.hlb;
LF  = params.vehicle.LF;
lr  = params.vehicle.lr;

alpha = 60;
if isfield(params, 'nlp') && isfield(params.nlp, 'alpha') && ~isempty(params.nlp.alpha)
    alpha = params.nlp.alpha;
end

signed_s = sgn * abs(s);
splus = smoothPlus(signed_s, alpha);
sminus = smoothPlus(-signed_s, alpha);

a = splus  + hlb * abs(k) * splus;   % up
c = sminus + hlb * abs(k) * sminus;  % down

b = smoothMax(-lr * k * splus, (LF + 0.5 * splus) * k * splus, alpha) ...
  + smoothMax(-LF * k * sminus, (lr + 0.5 * sminus) * k * sminus, alpha);

d = smoothMax( lr * k * splus, -(LF + 0.5 * splus) * k * splus, alpha) ...
  + smoothMax( LF * k * sminus, -(lr + 0.5 * sminus) * k * sminus, alpha);
end

function val = smoothPlus(x, alpha)
val = stableLogSumExp(0, x, alpha);
end

function val = smoothMax(a, b, alpha)
val = stableLogSumExp(a, b, alpha);
end

function val = stableLogSumExp(a, b, alpha)
m = max(a, b);
val = m + log(exp(alpha * (a - m)) + exp(alpha * (b - m))) / alpha;
end
