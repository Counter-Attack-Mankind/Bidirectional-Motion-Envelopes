function UpdateNfeConfig(task_id, nfe)
% LEGACY: Scheme2 now reads Scheme1's complete shared initial guess.
%UPDATENFECONFIG Update the task_id -> Nfe mapping used by Scheme2.

scheme_dir = fileparts(mfilename('fullpath'));
config_file = fullfile(scheme_dir, 'Nfe_config.txt');

task_id = round(task_id);
nfe = round(nfe);

if task_id < 1
    error('task_id must be a positive integer.');
end
if nfe < 2
    error('Nfe must be at least 2.');
end

if exist(config_file, 'file') == 2
    data = readmatrix(config_file);
    if isempty(data)
        data = zeros(0, 2);
    end
    if size(data, 2) < 2
        error('Invalid Nfe_config.txt format.');
    end
    data = data(:, 1:2);
else
    data = zeros(0, 2);
end

idx = find(data(:,1) == task_id, 1);
if isempty(idx)
    data(end+1, :) = [task_id, nfe];
    action_str = 'added';
else
    data(idx, 2) = nfe;
    action_str = 'updated';
end

data = sortrows(data, 1);
fid = fopen(config_file, 'w');
if fid < 0
    error('Cannot open Nfe_config.txt for writing.');
end
cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>

for i = 1:size(data, 1)
    fprintf(fid, '%d %d\r\n', data(i,1), data(i,2));
end

fprintf('Nfe config %s: task_id=%d, Nfe=%d\n', ...
    action_str, task_id, nfe);
end
