clear all; close all; clc

addpath(append(pwd, '/Functions'));

datespan = {'2024-01-02', '2024-01-15'};
folder_of_interest = 'BOXR_noise';
input = cell(2,1);
input{1} = datespan;
input{2} = folder_of_interest;

fprintf('\n')
fprintf('===================================================\n')
fprintf('Feel free to adjust the displayed regions before closing down the figure.\n')
fprintf('===================================================\n')
fprintf('\n')

compare_results(input);

