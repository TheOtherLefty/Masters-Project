% Simulation executable for quadrotor model
% Murray L Ireland
% September 2015

close all
clearvars -except h GlobalTime Count Run Runs SaveFile
clc

fprintf('-----------------------------------------------------\n')
fprintf('Simulation of an autonomous quadrotor vehicle\n')
fprintf('Murray L Ireland\n')
fprintf('September 2017\n')
fprintf('-----------------------------------------------------\n\n')

SimTime = tic;

% Load trajectory info


% AGENTS ------------------------------------------------------------------

% Environment
Environment = cEnvironment('Grid size',[5, 5]);

% Quadrotors
Agents.Quad = cQuadrotor('Quad',Environment,'Pose',[0 0 0 0 0 0]');

% Targets
NumTargets = 2;
Shapes = {'Cube','Ball','Pyramid'};
for i = 1:NumTargets
    Agents.Target(i) = cTarget(Shapes{i},Environment);
end

% SIMULATION ENGINE -------------------------------------------------------

% Simulation properties
t = 0;          % Initialise time (s)
tfin = 100;      % End time (s)
dt = 0.01;      % Solver increment (s)
tsamp = dt;    % Sample increment (s)
tsamp = 0.05;

% Initialise blackbox
Data = cBlackBox;

% Initialise
Sim = cSimEngine(Data,Environment,Agents,t,tfin,dt,tsamp);

% Simulation loop
fprintf('Running simulation...\n\n')
Data = Sim.SimLoop;
fprintf('\nSimulation complete\n')
fprintf('Time taken: %.2f s\n\n',toc(SimTime))

% Visualisations
Data.PlotData;



