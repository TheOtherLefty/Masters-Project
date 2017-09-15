% Generate excel row from saved data

function GenerateExcel(SaveFile)

% close all
% clearvars -except h GlobalTime Count Run Runs SaveFile
% clc

fprintf('Adding data to Excel file...\n\n')

% datafile = 'ModelPropsTestRun-05-12-15';
datafile = SaveFile;

load(datafile)

% excelfile = 'SimData-08-12-15';
excelfile = SaveFile;

% Setup Excel if it doesn't exist already
if ~exist([excelfile,'.xls'],'file')
    
    % Generic data
    MainHeaders = fieldnames(Props)';
    MainHeaders = ['Date','Time','Datafile',MainHeaders([1:2,7:9,3])];
    xlswrite(excelfile,MainHeaders,'Generic')
    
    % State times
    StateTimeHeaders = Props.AvgStateTimes(:,1)';
    StateTimeHeaders = ['Date','Time','Datafile',StateTimeHeaders];
    xlswrite(excelfile,StateTimeHeaders,'Avg State Times')
    
    % State distances
    StateDistanceHeaders = Props.AvgStateDistances(:,1)';
    StateDistanceHeaders = ['Date','Time','Datafile',StateDistanceHeaders];
    xlswrite(excelfile,StateDistanceHeaders,'Avg State Distances')
    
    % State transitions
    StateTransitions(:,1) = Props.ProbStateTransitions(:,1);
    StateTransitions(:,2) = {' -> '};
    StateTransitions(:,3) = Props.ProbStateTransitions(:,2);
    StateTransitionHeaders = strcat(StateTransitions(:,1),StateTransitions(:,2),StateTransitions(:,3));
    StateTransitionHeaders = StateTransitionHeaders';
    StateTransitionHeaders = ['Date','Time','Datafile',StateTransitionHeaders];
    xlswrite(excelfile,StateTransitionHeaders,'State Transition Probs')
    
end

% Read existing Excel file and identify number of entries
ExstData = xlsread(excelfile,'Generic');
i = size(ExstData,1) + 2;
range = ['A',num2str(i)];

% Current date and time
Clock = clock;
Date = [num2str(Clock(3)),'/',num2str(Clock(2)),'/',num2str(Clock(1))];
Time = [num2str(Clock(4)),':',num2str(Clock(5)),':',num2str(Clock(6))];

% Setup generic data
MainData = {Date,Time,datafile,...
    Props.NumRuns,Props.AvgMissionTime,...
    Props.ProbSystemsFault,Props.ProbActuatorFault,Props.ProbGrabberFault,Props.MissionSuccess};

% Setup state times
[~,ExstHeaders,~] = xlsread(excelfile,'Avg State Times','1:1');
StateTimeData = {Date,Time,datafile};
for j = 1:size(Props.AvgStateTimes,1)
    
    c = ismember(ExstHeaders,Props.AvgStateTimes(j,1));
    if ~isempty(c)
        StateTimeData{c} = Props.AvgStateTimes{j,2};
    else
        error('Problem')
    end
    
end

% Setup state distances
[~,ExstHeaders,~] = xlsread(excelfile,'Avg State Distances','1:1');
StateDistanceData = {Date,Time,datafile};
for j = 1:size(Props.AvgStateDistances,1)
    
    c = ismember(ExstHeaders,Props.AvgStateDistances(j,1));
    if ~isempty(c)
        StateDistanceData{c} = Props.AvgStateDistances{j,2};
    else
        error('Problem')
    end
    
end

% Setup state transitions
[~,ExstHeaders,~] = xlsread(excelfile,'State Transition Probs','1:1');
CurrentHeaders(:,1) = Props.ProbStateTransitions(:,1);
CurrentHeaders(:,2) = {' -> '};
CurrentHeaders(:,3) = Props.ProbStateTransitions(:,2);
CurrentHeaders = strcat(CurrentHeaders(:,1),CurrentHeaders(:,2),CurrentHeaders(:,3));
StateTransitionData = {Date,Time,datafile};
for j = 1:size(Props.ProbStateTransitions,1)
    
    c = ismember(ExstHeaders,CurrentHeaders(j));
    if ~isempty(c)
        StateTransitionData{c} = Props.ProbStateTransitions{j,3};
    else
        error('Problem')
    end
    
end

% Write data
xlswrite(excelfile,MainData,'Generic',range)
xlswrite(excelfile,StateTimeData,'Avg State Times',range)
xlswrite(excelfile,StateDistanceData,'Avg State Distances',range)
xlswrite(excelfile,StateTransitionData,'State Transition Probs',range)