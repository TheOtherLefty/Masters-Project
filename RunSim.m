function [Data,Sim,Environment,Fail] = RunSim

% close all
% clearvars -except h GlobalTime Count Run Runs SaveFile
% clc

fprintf('-----------------------------------------------------\n')
fprintf('Simulation of an autonomous quadrotor vehicle\n')
fprintf('Murray L Ireland\n')
fprintf('September 2015\n')
fprintf('-----------------------------------------------------\n\n')

SimTime = tic;

% AGENTS ------------------------------------------------------------------

% Environment
Environment = cEnvironment;

% Quadrotors
Pos = [-1.8795 -2.5936 -0.2000]'; % Initial position (m)
Att = [0 0 0.9305]'; % Initial attitude (rad)
NumQuads = 1;
for i = 1:NumQuads
    Agents.Quad(i) = cQuadrotor(['Quad',num2str(i)],Environment,...
        'Pose',[Pos; Att]);
end

% Targets
NumTargets = 2;
TPos = [-1.6312 -1.3453 -0.05
         1.5878 -2.1132 -0.05
        -1.1280 -1.6814 -0.05]';
TAtt = [0 0 -1.6742
        0 0 -0.8256
        0 0  0.6004]';
Shapes = {'Cube','Ball','Pyramid'};
for i = 1:NumTargets
    Agents.Target(i) = cTarget(Shapes{i},Environment,'Pose');%,...
%         'Pose',[TPos(:,i); TAtt(:,i)]);
end

% SIMULATION ENGINE -------------------------------------------------------

% Simulation properties
t = 0;          % Initialise time (s)
tfin = 400;      % End time (s)
dt = 0.01;      % Solver increment (s)
tsamp = dt;    % Sample increment (s)
tsamp = 0.1;

% Initialise blackbox
Data = cBlackBox;

% Initialise
Sim = cSimEngine(Data,Environment,Agents,t,tfin,dt,tsamp);

% Simulation loop
fprintf('Running simulation...\n\n')
Data = Sim.SimLoop;
fprintf('\nSimulation complete\n')
fprintf('Time taken: %.2f s\n\n',toc(SimTime))

Fail = 0;
if abs(Data.Time(end) - tfin) < 2
    Fail = 1;
end

% Visualisations
Data.PlotData;

end

