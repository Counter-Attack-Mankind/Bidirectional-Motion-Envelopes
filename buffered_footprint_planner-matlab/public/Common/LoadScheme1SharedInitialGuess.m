function [x, y, theta, v, a, phy, w, dt, shared_file] = ...
    LoadScheme1SharedInitialGuess(experiment_root, task_id)
%LOADSCHEME1SHAREDINITIALGUESS Load the exact Scheme1 shared NLP warm start.

global params

task_id = round(task_id);
shared_file = fullfile(experiment_root, 'Scheme1_GeoTime_MaxNLP', ...
    'Results', sprintf('task_%02d', task_id), 'InitialGuess', ...
    'shared_initial_guess.mat');
if exist(shared_file, 'file') ~= 2
    error('LoadScheme1SharedInitialGuess:MissingFile', ...
        ['Shared Scheme1 initial guess is missing for task_id=%d. ', ...
         'Run Scheme1 for this task before Scheme2. File: %s'], ...
        task_id, shared_file);
end

loaded = load(shared_file, 'shared');
if ~isfield(loaded, 'shared')
    error('LoadScheme1SharedInitialGuess:InvalidFile', ...
        'Expected variable shared in %s.', shared_file);
end
shared = loaded.shared;

if ~isfield(shared, 'task_id') || shared.task_id ~= task_id
    error('LoadScheme1SharedInitialGuess:TaskMismatch', ...
        'Shared initial guess task identifier does not match task_id=%d.', task_id);
end
requested_source = strtrim(getenv('EXPERIMENT_TASK_SOURCE'));
if ~isempty(requested_source) && isfield(shared, 'task_source') && ...
        ~isempty(shared.task_source) && ...
        ~strcmpi(requested_source, shared.task_source)
    error('LoadScheme1SharedInitialGuess:SourceMismatch', ...
        'Shared task source is %s, but the current source is %s.', ...
        shared.task_source, requested_source);
end
if isfield(shared, 'task_pose') && isfield(params, 'task')
    current_pose = [params.task.x0; params.task.y0; params.task.theta0; ...
        params.task.xf; params.task.yf; params.task.thetaf];
    if numel(shared.task_pose) ~= 6 || ...
            max(abs(shared.task_pose(:) - current_pose)) > 1e-12
        error('LoadScheme1SharedInitialGuess:TaskPoseMismatch', ...
            'Shared initial guess does not belong to the currently loaded task pose.');
    end
end

names = {'x','y','theta','v','a','phy','w','dt'};
for i = 1:numel(names)
    if ~isfield(shared, names{i})
        error('LoadScheme1SharedInitialGuess:MissingField', ...
            'Shared initial guess is missing field %s.', names{i});
    end
end

x = shared.x(:); y = shared.y(:); theta = shared.theta(:);
v = shared.v(:); a = shared.a(:); phy = shared.phy(:); w = shared.w(:);
dt = shared.dt(:);
nfe = numel(x);
if any([numel(y),numel(theta),numel(v),numel(a),numel(phy),numel(w)] ~= nfe) || ...
        numel(dt) ~= nfe - 1
    error('LoadScheme1SharedInitialGuess:InvalidDimensions', ...
        'Shared initial-guess dimensions are inconsistent.');
end
if any(~isfinite([x;y;theta;v;a;phy;w;dt]))
    error('LoadScheme1SharedInitialGuess:NonfiniteValue', ...
        'Shared initial guess contains a non-finite value.');
end

fprintf('Scheme1 shared initial guess loaded: task_id=%d, Nfe=%d\n', ...
    task_id, nfe);
end
