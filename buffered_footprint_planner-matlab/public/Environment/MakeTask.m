%% ===== 当前目录 =====
env_dir = fileparts(mfilename('fullpath'));

addpath(fullfile(env_dir, 'Utilities'));
addpath(fullfile(env_dir, 'Common'));
addpath(fullfile(env_dir, 'CaseData'));
addpath(fullfile(env_dir, 'Data_test'));
addpath(fullfile(env_dir, 'HybridA'));
addpath(fullfile(env_dir, 'ConvertTraj'));

%% ===== 任务配置与初始化 =====
task_id = 3;
task_file = fullfile(env_dir,'real',sprintf('%d.mat', task_id));
%task_file = fullfile(env_dir,'Data_test',sprintf('%d.mat', task_id));
InitializeParams();
if exist(task_file, 'file') ~= 2
    MakeTaskByMouse(task_file);
end

%DrawEnvironment(task_file);
%EditTaskObstacles(task_file, task_file);
modify_target(task_file);
DrawEnvironment(task_file);