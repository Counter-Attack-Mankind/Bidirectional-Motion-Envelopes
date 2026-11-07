function UpdateUnifiedResults(output_csv, scheme_dir, result)
%UPDATEUNIFIEDRESULTS Update one result in the root 13-column CSV file.
% Each scheme uses six columns and adjacent schemes are separated by one
% empty column. Collision percentages are written with two decimal places.

scheme_folders = { ...
    'Scheme1_GeoTime_MaxNLP', ...
    'Scheme2_body_only_baseline'};
[~, scheme_folder] = fileparts(scheme_dir);
scheme_index = find(strcmp(scheme_folder, scheme_folders), 1);
if isempty(scheme_index)
    error('UpdateUnifiedResults:UnknownScheme', ...
        'Cannot map scheme directory to results.csv: %s', scheme_dir);
end

headers = unifiedHeaders();
if exist(output_csv, 'file') == 2
    raw = readcell(output_csv, 'Delimiter', ',');
    if isempty(raw)
        error('UpdateUnifiedResults:InvalidTable', ...
            'Existing results.csv is empty.');
    end
    actual_headers = string(raw(1,:));
    actual_headers(ismissing(actual_headers)) = "";
    if size(raw,2) ~= 13 || ...
            ~isequal(actual_headers,string(headers))
        error('UpdateUnifiedResults:InvalidTable', ...
            'Existing results.csv does not have the expected 13-column layout.');
    end
    rows = raw(2:end,:);
else
    rows = cell(0,13);
end

task_ids = nan(size(rows,1),1);
for r = 1:size(rows,1)
    task_ids(r) = numericCell(rows{r,1});
end
row_index = find(task_ids == result.task_id,1);
if isempty(row_index)
    new_row = cell(1,13);
    for c = [1,8]
        new_row{c} = result.task_id;
    end
    for c = [2:6,9:13]
        new_row{c} = NaN;
    end
    rows(end+1,:) = new_row;
    row_index = size(rows,1);
end

base_column = 1+(scheme_index-1)*7;
rows{row_index,base_column} = result.task_id;
rows{row_index,base_column+1} = double(result.success);
rows{row_index,base_column+2} = result.inf_pr_0;
rows{row_index,base_column+3} = result.ipopt_iterations;
rows{row_index,base_column+4} = result.ipopt_cpu_sec;
rows{row_index,base_column+5} = round(result.collision_percent,2);

ids = nan(size(rows,1),1);
for r = 1:size(rows,1)
    ids(r) = numericCell(rows{r,1});
end
[~,order] = sort(ids);
writeUnifiedCsv(output_csv,headers,rows(order,:));
fprintf('Unified evaluation table updated: %s\n',output_csv);
end

function headers = unifiedHeaders()
headers = cell(1,13);
for scheme_index = 1:2
    base = 1+(scheme_index-1)*7;
    headers{base} = sprintf('scheme%d_task_id',scheme_index);
    headers{base+1} = 'success';
    headers{base+2} = 'inf_pr_0';
    headers{base+3} = 'ipopt_iterations';
    headers{base+4} = 'ipopt_cpu_time';
    headers{base+5} = 'collision_percent';
    if scheme_index < 2
        headers{base+6} = '';
    end
end
end

function value = numericCell(item)
if isnumeric(item) || islogical(item)
    value = double(item);
else
    value = str2double(string(item));
end
end

function writeUnifiedCsv(filename,headers,rows)
fid = fopen(filename,'w');
if fid < 0
    error('UpdateUnifiedResults:CannotWriteCsv', ...
        'Cannot create unified results file: %s',filename);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',strjoin(headers,','));
task_columns = [1,8];
success_columns = [2,9];
inf_pr_columns = [3,10];
iteration_columns = [4,11];
cpu_columns = [5,12];
collision_columns = [6,13];
for r = 1:size(rows,1)
    fields = cell(1,13);
    for c = 1:13
        if c == 7
            fields{c} = '';
            continue;
        end
        value = numericCell(rows{r,c});
        if ~isfinite(value)
            fields{c} = 'NaN';
        elseif any(c == task_columns) || any(c == success_columns)
            fields{c} = sprintf('%d',round(value));
        elseif any(c == inf_pr_columns)
            fields{c} = sprintf('%.3f',value);
        elseif any(c == iteration_columns)
            fields{c} = sprintf('%d',round(value));
        elseif any(c == cpu_columns)
            fields{c} = sprintf('%.6f',value);
        elseif any(c == collision_columns)
            fields{c} = sprintf('%.2f',value);
        end
    end
    fprintf(fid,'%s\n',strjoin(fields,','));
end
end
