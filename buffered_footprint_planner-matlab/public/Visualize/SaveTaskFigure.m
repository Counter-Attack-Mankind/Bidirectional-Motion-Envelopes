function SaveTaskFigure(fig_handle,image_suffix)
%SAVETASKFIGURE Save a task plot in the scheme-level image directory.
% EFboxs  -> <scheme_dir>/EFboxs_photo/task_XX.png
% TrueBody -> <scheme_dir>/vehiclesbody_photo/task_XX.png

global params

if nargin < 1 || isempty(fig_handle)
    fig_handle = gcf;
end
if nargin < 2 || isempty(image_suffix)
    error('SaveTaskFigure requires image_suffix.');
end
if ~isfield(params,'io') || ...
        ~isfield(params.io,'scheme_dir') || ...
        ~isfield(params.io,'task_id')
    warning('SaveTaskFigure:MissingContext', ...
        'params.io.scheme_dir or params.io.task_id is missing. Figure not saved.');
    return;
end

scheme_dir = params.io.scheme_dir;
task_id = params.io.task_id;
switch lower(strtrim(image_suffix))
    case 'efboxs'
        output_folder = fullfile(scheme_dir,'EFboxs_photo');
    case 'truebody'
        output_folder = fullfile(scheme_dir,'vehiclesbody_photo');
    otherwise
        output_folder = fullfile(scheme_dir,'figures');
end
if exist(output_folder,'dir') ~= 7
    mkdir(output_folder);
end

file_path = fullfile(output_folder,sprintf('task_%02d.png',task_id));
exportgraphics(fig_handle,file_path,'Resolution',300);
fprintf('Figure saved: %s\n',file_path);
end

