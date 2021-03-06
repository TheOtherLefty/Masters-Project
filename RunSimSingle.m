% Simulation executable for quadrotor model
% Murray L Ireland
% September 2015

% -----------------------
%
% CURRENT STATUS:
% Quad moves to grid point and then checks its current state against the
% table of all possible states. It then moves in the direction specified by
% the table of transitions.
%
% NEED TO:
% Make sure quad only picks one state to move to. Additional states to
% include in conditions are the mode and battery.
%
% -----------------------

close all
clearvars -except h GlobalTime Count Run Runs SaveFile
clc

fprintf('-----------------------------------------------------\n')
fprintf('Simulation of an autonomous quadrotor vehicle\n')
fprintf('Murray L Ireland\n')
fprintf('September 2017\n')
fprintf('-----------------------------------------------------\n\n')

SimTime = tic;


% Load decision info
DecisionsFile = 'scenario3b_5x5_4_3'; % Initial state: 303
[States, Transitions] = LoadDecisions(DecisionsFile);

% AGENTS ------------------------------------------------------------------

% Environment
Environment = cEnvironment('Grid size',[5, 5]);

% Quadrotors
Agents.Quad = cQuadrotor('Quad',Environment,'Pose',[0 0 0 0 0 0]',...
    'States', States, 'Transitions', Transitions);

% Targets
NumTargets = 3;
Shapes = {'Cube','Ball','Pyramid'};
for i = 1:NumTargets
    Agents.Target(i) = cTarget(Shapes{i},Environment);
end

% SIMULATION ENGINE -------------------------------------------------------

% Simulation properties
t = 0;          % Initialise time (s)
tfin = 500;      % End time (s)
dt = 0.002;      % Solver increment (s)
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



