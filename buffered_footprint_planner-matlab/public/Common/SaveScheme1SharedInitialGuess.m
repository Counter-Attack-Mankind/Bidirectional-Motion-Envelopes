function shared_file = SaveScheme1SharedInitialGuess(scheme_dir, task_id, source_matfile)
%SAVESCHEME1SHAREDINITIALGUESS Save the exact shared variables written for NLP1.

if nargin < 3 || isempty(source_matfile)
    source_matfile = fullfile(scheme_dir, 'written_initial_guess_data.mat');
end
if exist(source_matfile, 'file') ~= 2
    error('SaveScheme1SharedInitialGuess:MissingSource', ...
        'Scheme1 initial-guess data does not exist: %s', source_matfile);
end

loaded = load(source_matfile, 'data');
if ~isfield(loaded, 'data')
    error('SaveScheme1SharedInitialGuess:InvalidSource', ...
        'Expected variable data in %s.', source_matfile);
end

data = loaded.data;
names = {'x','y','theta','v','a','phy','w','dt'};
shared = struct();
shared.schema_version = 1;
shared.task_id = round(task_id);
shared.task_source = strtrim(getenv('EXPERIMENT_TASK_SOURCE'));
if isempty(shared.task_source)
    shared.task_source = 'real';
end
shared.source_scheme = 'Scheme1_GeoTime_MaxNLP';
if isfield(data, 'meta') && all(isfield(data.meta, ...
        {'x0','y0','theta0','xf','yf','thetaf'}))
    shared.task_pose = [data.meta.x0; data.meta.y0; data.meta.theta0; ...
        data.meta.xf; data.meta.yf; data.meta.thetaf];
end

for i = 1:numel(names)
    name = names{i};
    if ~isfield(data, name)
        error('SaveScheme1SharedInitialGuess:MissingField', ...
            'Field %s is missing from %s.', name, source_matfile);
    end
    values = data.(name)(:);
    if isempty(values) || any(~isfinite(values))
        error('SaveScheme1SharedInitialGuess:InvalidField', ...
            'Field %s contains no usable finite values.', name);
    end
    shared.(name) = values;
end

shared.nfe = numel(shared.x);
for i = 2:7
    if numel(shared.(names{i})) ~= shared.nfe
        error('SaveScheme1SharedInitialGuess:InvalidDimensions', ...
            'Shared field %s must contain Nfe values.', names{i});
    end
end
if numel(shared.dt) ~= shared.nfe - 1
    error('SaveScheme1SharedInitialGuess:InvalidDimensions', ...
        'Shared dt must contain Nfe-1 values.');
end

paths = PrepareStrategyRunFolders(scheme_dir, shared.task_id);
shared_file = fullfile(paths.initial_guess, 'shared_initial_guess.mat');
save(shared_file, 'shared');
fprintf('Scheme1 shared initial guess saved: %s\n', shared_file);
end
