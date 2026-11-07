function VerifySmoothErrorBounds()
% LEGACY DIAGNOSTIC: verifies bounds for the retired LSE approximation.
% VerifySmoothErrorBounds
% 验证 problem.txt 中 log-sum-exp / softplus 平滑外扩尺度误差界。
%
% 直接运行:
%   VerifySmoothErrorBounds
%
% 输出内容:
%   1) softplus 对 s+、s- 的误差是否落在 (0, ln2/alpha]
%   2) LSE_alpha(a,b) 对 max(a,b) 的误差是否落在 [0, ln2/alpha]
%   3) eup, edown, eleft, eright 的实测误差是否小于理论上界
%   4) v > 0, kappa > 0 典型情形下 eleft 的直接拆分验证

clc;

%% Vehicle / smoothing parameters
LF  = 2.96;
lr  = 0.929;
hlb = 0.96;
dt  = 0.25;

alphas = [5, 10, 20, 50, 100, 200];

v_grid     = linspace(-5, 5, 401);
kappa_grid = linspace(-0.7, 0.7, 401);
[V, KAPPA] = ndgrid(v_grid, kappa_grid);
S = V * dt;

fprintf('============================================================\n');
fprintf('Verify log-sum-exp smooth error bounds\n');
fprintf('LF = %.6g, lr = %.6g, hlb = %.6g, dt = %.6g\n', LF, lr, hlb, dt);
fprintf('samples = %d\n', numel(S));
fprintf('============================================================\n\n');

for alpha = alphas
    eps_alpha = log(2) / alpha;

    [orig, smooth, aux] = computeAll(S, KAPPA, alpha, LF, lr, hlb);
    bounds = computeBounds(aux, KAPPA, alpha, LF, lr, hlb);

    err_splus  = aux.splus_hat  - aux.splus;
    err_sminus = aux.sminus_hat - aux.sminus;

    err_eup    = smooth.eup    - orig.eup;
    err_edown  = smooth.edown  - orig.edown;
    err_eleft  = smooth.eleft  - orig.eleft;
    err_eright = smooth.eright - orig.eright;

    fprintf('alpha = %.6g, ln2/alpha = %.12g\n', alpha, eps_alpha);
    reportBound('softplus s+',  min(err_splus(:)),  max(err_splus(:)),  eps_alpha);
    reportBound('softplus s-',  min(err_sminus(:)), max(err_sminus(:)), eps_alpha);

    checkLSEBound(alpha);

    reportBound('eup',    min(err_eup(:)),    max(err_eup(:)),    max(bounds.eup(:)));
    reportBound('edown',  min(err_edown(:)),  max(err_edown(:)),  max(bounds.edown(:)));
    reportBound('eleft',  min(err_eleft(:)),  max(err_eleft(:)),  max(bounds.eleft(:)));
    reportBound('eright', min(err_eright(:)), max(err_eright(:)), max(bounds.eright(:)));

    fprintf('  max(abs(eleft error)  - bound) = %.3e\n', max(abs(err_eleft(:))  - bounds.eleft(:)));
    fprintf('  max(abs(eright error) - bound) = %.3e\n', max(abs(err_eright(:)) - bounds.eright(:)));

    if alpha == alphas(1)
        showTypicalCase(alpha, LF, lr, hlb, dt);
    end

    fprintf('\n');
end

fprintf('结论: 若所有 max(error - bound) <= 0 且基础误差区间满足要求，数值验证支持推导的误差界。\n');

end

function [orig, smooth, aux] = computeAll(S, KAPPA, alpha, LF, lr, hlb)
splus  = max(S, 0);
sminus = max(-S, 0);

splus_hat  = softplusStable(alpha * S) / alpha;
sminus_hat = softplusStable(-alpha * S) / alpha;

orig.eup   = (1 + hlb * abs(KAPPA)) .* splus;
orig.edown = (1 + hlb * abs(KAPPA)) .* sminus;

orig.eleft = max(-lr .* KAPPA .* splus, ...
                 (LF + 0.5 .* splus) .* KAPPA .* splus) ...
           + max(-LF .* KAPPA .* sminus, ...
                 (lr + 0.5 .* sminus) .* KAPPA .* sminus);

orig.eright = max(lr .* KAPPA .* splus, ...
                  -(LF + 0.5 .* splus) .* KAPPA .* splus) ...
            + max(LF .* KAPPA .* sminus, ...
                  -(lr + 0.5 .* sminus) .* KAPPA .* sminus);

smooth.eup   = (1 + hlb * abs(KAPPA)) .* splus_hat;
smooth.edown = (1 + hlb * abs(KAPPA)) .* sminus_hat;

smooth.eleft = lse2(alpha, -lr .* KAPPA .* splus_hat, ...
                           (LF + 0.5 .* splus_hat) .* KAPPA .* splus_hat) ...
             + lse2(alpha, -LF .* KAPPA .* sminus_hat, ...
                           (lr + 0.5 .* sminus_hat) .* KAPPA .* sminus_hat);

smooth.eright = lse2(alpha, lr .* KAPPA .* splus_hat, ...
                            -(LF + 0.5 .* splus_hat) .* KAPPA .* splus_hat) ...
              + lse2(alpha, LF .* KAPPA .* sminus_hat, ...
                            -(lr + 0.5 .* sminus_hat) .* KAPPA .* sminus_hat);

aux.splus       = splus;
aux.sminus      = sminus;
aux.splus_hat   = splus_hat;
aux.sminus_hat  = sminus_hat;
aux.delta_splus = splus_hat - splus;
aux.delta_sminus = sminus_hat - sminus;
end

function bounds = computeBounds(aux, KAPPA, alpha, LF, lr, hlb)
eps_alpha = log(2) / alpha;

bounds.eup   = (1 + hlb * abs(KAPPA)) .* eps_alpha;
bounds.edown = (1 + hlb * abs(KAPPA)) .* eps_alpha;

left_splus_geometry = abs(KAPPA) .* max(lr, LF + aux.splus_hat) ...
                    .* aux.delta_splus;
left_sminus_geometry = abs(KAPPA) .* max(LF, lr + aux.sminus_hat) ...
                     .* aux.delta_sminus;

right_splus_geometry = abs(KAPPA) .* max(lr, LF + aux.splus_hat) ...
                     .* aux.delta_splus;
right_sminus_geometry = abs(KAPPA) .* max(LF, lr + aux.sminus_hat) ...
                      .* aux.delta_sminus;

bounds.eleft  = left_splus_geometry  + left_sminus_geometry  + 2 * eps_alpha;
bounds.eright = right_splus_geometry + right_sminus_geometry + 2 * eps_alpha;
end

function checkLSEBound(alpha)
eps_alpha = log(2) / alpha;
a = linspace(-10, 10, 301);
b = linspace(-10, 10, 301);
[A, B] = ndgrid(a, b);
err = lse2(alpha, A, B) - max(A, B);
fprintf('  LSE(a,b): min error = %.3e, max error = %.12g, bound = %.12g\n', ...
        min(err(:)), max(err(:)), eps_alpha);
end

function showTypicalCase(alpha, LF, lr, hlb, dt)
v = 2.0;
kappa = 0.35;
s = v * dt;

[orig, smooth, aux] = computeAll(s, kappa, alpha, LF, lr, hlb);

eleft_explicit = (LF + 0.5 * aux.splus) * kappa * aux.splus;

splus_geometry = (LF + 0.5 * aux.splus_hat) * kappa * aux.splus_hat ...
               - (LF + 0.5 * aux.splus) * kappa * aux.splus;
sminus_geometry = (lr + 0.5 * aux.sminus_hat) * kappa * aux.sminus_hat;

lse_error_splus = lse2(alpha, -lr * kappa * aux.splus_hat, ...
                              (LF + 0.5 * aux.splus_hat) * kappa * aux.splus_hat) ...
                - max(-lr * kappa * aux.splus_hat, ...
                      (LF + 0.5 * aux.splus_hat) * kappa * aux.splus_hat);

lse_error_sminus = lse2(alpha, -LF * kappa * aux.sminus_hat, ...
                               (lr + 0.5 * aux.sminus_hat) * kappa * aux.sminus_hat) ...
                 - max(-LF * kappa * aux.sminus_hat, ...
                       (lr + 0.5 * aux.sminus_hat) * kappa * aux.sminus_hat);

direct_sum = splus_geometry + sminus_geometry + lse_error_splus + lse_error_sminus;

fprintf('\n  Typical case v > 0, kappa > 0 for eleft:\n');
fprintf('    s = %.6g, s+ = %.6g, s- = %.6g\n', s, aux.splus, aux.sminus);
fprintf('    original max branches: plus uses (LF+0.5*s+)*kappa*s+, minus both zero\n');
fprintf('    explicit eleft = %.12g, formula eleft = %.12g\n', eleft_explicit, orig.eleft);
fprintf('    eleft_hat - eleft = %.12g\n', smooth.eleft - orig.eleft);
fprintf('      s+ replacement geometry error = %.12g\n', splus_geometry);
fprintf('      s- replacement geometry error = %.12g\n', sminus_geometry);
fprintf('      LSE error, s+ max = %.12g\n', lse_error_splus);
fprintf('      LSE error, s- max = %.12g\n', lse_error_sminus);
fprintf('      decomposed sum = %.12g\n', direct_sum);
end

function y = softplusStable(x)
y = max(x, 0) + log1p(exp(-abs(x)));
end

function y = lse2(alpha, a, b)
aa = alpha .* a;
bb = alpha .* b;
c = max(aa, bb);
y = (c + log(exp(aa - c) + exp(bb - c))) ./ alpha;
end

function reportBound(name, min_error, max_error, bound)
tol = 1e-10;
ok = max_error <= bound + tol && min_error >= -tol;
fprintf('  %-14s min error = %+ .3e, max error = %.12g, bound = %.12g, %s\n', ...
        name, min_error, max_error, bound, passFail(ok));
end

function s = passFail(ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
end
