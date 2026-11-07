function paths = PrepareStrategyRunFolders(strategy_dir, task_id)
paths.root = fullfile(strategy_dir, 'Results', sprintf('task_%02d', task_id));
paths.initial_guess = fullfile(paths.root, 'InitialGuess');
paths.optimized_variables = fullfile(paths.root, 'OptimizedVariables');

if ~isfolder(paths.initial_guess)
    mkdir(paths.initial_guess);
end
if ~isfolder(paths.optimized_variables)
    mkdir(paths.optimized_variables);
end
end

