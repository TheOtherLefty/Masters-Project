% Monte Carlo simulation script based on RunSimSingle.
% Runs 1000 instances of the simulation recording run time and mission
% status.

close all
clearvars -except h GlobalTime Count Run Runs SaveFile
clc

fprintf('-----------------------------------------------------\n')
fprintf('Monte Carlo Simulation of UAV\n')
fprintf('Douglas H Fraser\n')
fprintf('March 2019\n')
fprintf('-----------------------------------------------------\n\n')

SimTime = tic;
NumSims = 1000;

fprintf('Running %d simulations...\n\n', NumSims)
results = ["Sim #","Time","Status","Battery Used","Remaining Objects","Initial Search Mode","Final Search Mode","Details"];

% Simulation Loop --------------------------------------------------
for simNum = 1:NumSims
    close all
    clearvars -except h GlobalTime row results simNum NumSims Count Run Runs SaveFile

    % Initialize controller and environment for simulation.
    DecisionsFile = 'ControllerV2/scenario3b_5x5_1'; % Initial state: 303
    [States, Transitions] = LoadDecisions(DecisionsFile);

    Environment = cEnvironment('Grid size',[5, 5]);

    Agents.Quad = cQuadrotor('Quad',Environment,'Pose',[0 0 0 0 0 0]',...
        'States', States, 'Transitions', Transitions);

    % Targets
    NumTargets = 3;
    Shapes = {'Cube','Ball','Pyramid'};
    for i = 1:NumTargets
        Agents.Target(i) = cTarget(Shapes{i},Environment);
    end

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
    fprintf('Running instance #%d of %d\n', simNum, NumSims)
    Data = Sim.SimLoop;
    fprintf('\nInstance #%d of %d complete\n', simNum, NumSims)
    fprintf('Time taken: %.2f s\n\n',Sim.Time)
    %fprintf('Mission Status:',  ,'\n\n',toc(SimTime))

    Quad = Sim.Agents.Quad;

    TargetsRemaining = Quad.NumTargets - Quad.TargetCount;

    if Quad.MissionComplete
        status = "Success";
    else
        status = "Failed";
    end
    
    entry = [mat2str(simNum), mat2str(Sim.Time), mat2str(Quad.MissionComplete), mat2str(Quad.BatteryUsage), mat2str(TargetsRemaining), Quad.InitialSearchType, Quad.SearchType, Quad.FailureType];
    
    results = [results;entry];
    
    %if mod(row,10)
    %    writetable(cell2table(num2cell(results)),"MCResults.xlsx",'Sheet',1,'Range',strcat("A",mat2str(row+1),":H",mat2str(row+11)))
    %    results = [];
    %else
    %    row = row + 1;
    %end
end

fprintf("Simulations complete; writing results to CSV file.\n")

writetable(cell2table(num2cell(results)),"MCResults.xlsx")

fprintf("Complete: Results table successfully written.\n")

