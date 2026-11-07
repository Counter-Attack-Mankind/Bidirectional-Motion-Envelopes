function max_error = AssertSharedInitialGuessConsistency( ...
    shared_file, scheme2_matfile, tolerance)
%ASSERTSHAREDINITIALGUESSCONSISTENCY Compare every shared NLP initial value.

if nargin < 3 || isempty(tolerance)
    tolerance = 1e-9;
end
if ~(isscalar(tolerance) && isfinite(tolerance) && tolerance >= 0)
    error('AssertSharedInitialGuessConsistency:InvalidTolerance', ...
        'Tolerance must be a finite nonnegative scalar.');
end

scheme1_loaded = load(shared_file, 'shared');
scheme2_loaded = load(scheme2_matfile, 'data');
if ~isfield(scheme1_loaded, 'shared') || ~isfield(scheme2_loaded, 'data')
    error('AssertSharedInitialGuessConsistency:InvalidFile', ...
        'Expected shared/data variables in the two initial-guess files.');
end
shared = scheme1_loaded.shared;
data = scheme2_loaded.data;

if ~isfield(data, 'meta') || ~isfield(data.meta, 'task_id') || ...
        data.meta.task_id ~= shared.task_id
    error('AssertSharedInitialGuessConsistency:TaskMismatch', ...
        'Scheme1 and Scheme2 initial guesses have different task identifiers.');
end

names = {'x','y','theta','v','a','phy','w','dt'};
errors = zeros(size(names));
for i = 1:numel(names)
    name = names{i};
    lhs = shared.(name)(:);
    rhs = data.(name)(:);
    if numel(lhs) ~= numel(rhs)
        error('AssertSharedInitialGuessConsistency:SizeMismatch', ...
            'Shared field %s has different sizes in Scheme1 and Scheme2.', name);
    end
    errors(i) = max(abs(lhs - rhs));
    if isempty(errors(i))
        errors(i) = 0;
    end
end
max_error = max(errors);

fprintf('\n========== Shared Initial Guess Consistency ==========\n');
for i = 1:numel(names)
    fprintf('%-8s max abs error: %.3e\n', names{i}, errors(i));
end
fprintf('overall maximum error: %.3e (tolerance %.3e)\n', ...
    max_error, tolerance);
fprintf('======================================================\n\n');

if ~isfinite(max_error) || max_error > tolerance
    error('AssertSharedInitialGuessConsistency:Mismatch', ...
        ['Scheme1/Scheme2 shared initial values differ: ', ...
         'maximum error %.12g exceeds tolerance %.12g.'], ...
        max_error, tolerance);
end
end
