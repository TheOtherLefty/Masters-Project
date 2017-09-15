% Run loop to obtain verification data

close all
clear
clc

% Settings
Runs = 1000;

% Set save file
SaveFile = 'SimData-12-05-16';
% SaveFile = 'ModelPropsTestRun-14-01-16';
% SaveFile = 'ModelPropsTestRun-22-12-15';

% Waitbar
h = waitbar(0,'Running simulations...');

% Runtime
GlobalTime = tic;

Ind = 1;

for Run = 1:Runs
    
    % Update waitbar
    waitbar(Run/Runs,h,['Run ',num2str(Run),' of ',num2str(Runs)]);
    
    % Update message
    fprintf('Run %d of %d\n\n',Run,Runs)
    
    % Run simulation script
    [Data,Sim,Environment,Fail] = RunSim;
    
    % Run data collection script
    if ~Fail
        Ind = GetProperties(SaveFile,Data,Sim,Environment);
    end
    
    % Run data averaging every n runs
    n = 50;
    if rem(Ind,n) < 1e-2
        AverageProperties(SaveFile)
        GenerateExcel(SaveFile)
    end
    
end

% AverageProperties(SaveFile)
% GenerateExcel(SaveFile)

fprintf('Simulation runs complete\n')
fprintf('Time taken: %.2f mins\n\n',toc(GlobalTime)/60)

% Close waitbar
close(h)