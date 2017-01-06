clear;
close all;
clc;

rng(1); % fix random seed

addpath(genpath('helpers'));
addpath(genpath('testing'));
addpath(genpath('visualization'));

% load parameter struct
params = loadParameters();

% code profiling
if params.perf.profiling
    profile on;
end

% run VO pipeline
runVOPipeline(params);

% view profiling results
if params.perf.profiling
    profile viewer;
end