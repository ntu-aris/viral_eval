clear all;
close all;

tic

path        = matlab.desktop.editor.getActiveFilename;
this_dir    = path(1: end - length(mfilename) - 2);
cd(this_dir);

tests       = dir([this_dir 'result_eee*']);
tests_count = length(tests);

ATE_POSE    = cell(tests_count, 2);

t_shift     = 0.0;

fprintf('Number of tests: %d\n', length(tests));

% delete(gcp)
% parpool(6);

for n=1:tests_count
% parfor n=1:tests_count
    
    P_h_ate = evaluate_one(n, tests(n));
            
    ATE_POSE(n, :) = {tests(n).name, P_h_ate};
end


save('evaluation_result.mat', 'ATE_POSE');

ATE_POSE

toc