% Simulation executable for quadrotor model
% Murray L Ireland
% September 2015

close all
clearvars -except h GlobalTime Count Run Runs SaveFile
clc

fprintf('-----------------------------------------------------\n')
fprintf('Simulation of an autonomous quadrotor vehicle\n')
fprintf('Murray L Ireland\n')
fprintf('September 2015\n')
fprintf('-----------------------------------------------------\n\n')

SimTime = tic;

% Manual or random properties?
Manual = 1;

% Run instance from recorded data?
Inst = 0;

if Inst > 0
    load('ModelPropsTestRun-05-12-15_Fails')
end

% AGENTS ------------------------------------------------------------------

% Environment
if Inst > 0
    DropSite = BadProps(Inst).InitCond.DropSite(1:3);
else
    DropSite = [1.3303 -1.3307 0]';
end

if Manual
    Environment = cEnvironment('Dropsite',DropSite);
else
    Environment = cEnvironment;
end

% Quadrotors
NumQuads = 1;
if Inst > 0
    Pos = BadProps(Inst).InitCond.Quad(1:3);
    Att = BadProps(Inst).InitCond.Quad(4:6);
else
    Pos = [-1.8795 -2.5936 -0.2000]'; % Initial position (m)
    Att = [0 0 0.9305]'; % Initial attitude (rad)
end

if Manual
    for i = 1:NumQuads
        Agents.Quad(i) = cQuadrotor(['Quad',num2str(i)],Environment,...
            'Pose',[Pos; Att]);
    end
else
    for i = 1:NumQuads
        Agents.Quad(i) = cQuadrotor(['Quad',num2str(i)],Environment);
    end
end

Manual = 0;

% Targets
NumTargets = 2;
if Inst > 0
    for i = 1:NumTargets
        TPos(:,i) = BadProps(Inst).InitCond.(['Target',num2str(i)])(1:3);
        TAtt(:,i) = BadProps(Inst).InitCond.(['Target',num2str(i)])(4:6);
    end
else
    TPos = [-0.4292 1.1183 -0.05
            -1.2244 2.4054 -0.05
            -1.2091 1.9678 -0.05]';
    TAtt = [0 0 -2.5815
            0 0  2.5964
            0 0 -0.6785]';
end
Shapes = {'Cube','Ball','Pyramid'};
if Manual
    for i = 1:NumTargets
        Agents.Target(i) = cTarget(Shapes{i},Environment,...
            'Pose',[TPos(:,i); TAtt(:,i)]);
    end
else
    for i = 1:NumTargets
        Agents.Target(i) = cTarget(Shapes{i},Environment);
    end
end

% SIMULATION ENGINE -------------------------------------------------------

% Simulation properties
t = 0;          % Initialise time (s)
tfin = 1000;      % End time (s)
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



