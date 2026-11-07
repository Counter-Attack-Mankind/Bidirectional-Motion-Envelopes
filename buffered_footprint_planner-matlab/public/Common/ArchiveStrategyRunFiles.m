function ArchiveStrategyRunFiles(scheme_dir, task_id, phase)

if nargin < 3
    phase = 'all';
end

% 当前任务的正式归档目录
paths = PrepareStrategyRunFolders(scheme_dir, task_id);

%% =========================
% 归档 Initial Guess
% ==========================

if any(strcmpi(phase, {'all','initial'}))

    copyIfExists(fullfile(scheme_dir, 'ig.INIVAL'), ...
                 paths.initial_guess);

    copyIfExists(fullfile(scheme_dir, 'PV'), ...
                 paths.initial_guess);

    copyIfExists(fullfile(scheme_dir, 'OBCAData.dat'), ...
                 paths.initial_guess);

    copyIfExists(fullfile(scheme_dir, 'PPP'), ...
                 paths.initial_guess);

    copyIfExists(fullfile(scheme_dir, 'Area'), ...
                 paths.initial_guess);

    copyIfExists(fullfile(scheme_dir, 'written_initial_guess_data.mat'), ...
                 paths.initial_guess);
end

%% =========================
% 归档优化结果
% ==========================

result_dir = fullfile(scheme_dir, 'runtime');

if isfolder(result_dir) && any(strcmpi(phase, {'all','optimized'}))

    files = dir(fullfile(result_dir, '*.txt'));

    for i = 1:numel(files)
        copyfile( ...
            fullfile(files(i).folder, files(i).name), ...
            paths.optimized_variables);
    end

    writeCombinedOptimizedVariables( ...
        paths.optimized_variables, task_id);
end

end


%% =========================================================
% 辅助函数：文件存在时复制
% =========================================================
function copyIfExists(src, dst)

if exist(src, 'file') == 2
    copyfile(src, dst);
end

end


%% =========================================================
% 辅助函数：汇总所有优化变量
% =========================================================
function writeCombinedOptimizedVariables(result_dir, task_id)

names = { ...
    'x','y','theta','v','a','phy','w', ...
    'dt','s','splus','sminus','k', ...
    'up','down','left','right'};

out_file = fullfile( ...
    result_dir, ...
    sprintf('optimized_variables_task_%02d.txt', task_id));

fid = fopen(out_file, 'w');

if fid < 0
    error('Cannot create file: %s', out_file);
end

cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'task_id %d\r\n', task_id);

for i = 1:numel(names)

    file = fullfile(result_dir, [names{i}, '.txt']);

    if exist(file, 'file') ~= 2
        continue;
    end

    values = load(file);

    fprintf(fid, '\r\n[%s]\r\n', names{i});

    for j = 1:numel(values)
        fprintf(fid, '%d %.12f\r\n', j, values(j));
    end

end

end
