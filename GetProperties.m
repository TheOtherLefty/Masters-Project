% Collect properties from continuous-time model for use in finite-state
% model

function i = GetProperties(SaveFile,Data,Sim,Environment)

% close all
% clearvars -except h Count GlobalTime Run Runs SaveFile Data Agents Environment Sim
% clc

fprintf('Getting properties...\n\n')

% Load dataset
if ~exist([SaveFile,'.mat'],'file')
    i = 1;
else
    load(SaveFile)
    i = length(AllProps) + 1;
end

%% BASIC PROPERTIES

% Save run number
AllProps(i).Run = i;

% Get mission time
AllProps(i).MissionTime = Data.Time(end);

% Get mission outcome
if Sim.Agents.Quad(1).MissionComplete
    AllProps(i).MissionOutcome = 1;
else
    AllProps(i).MissionOutcome = 0;
end

% Initial conditions
AllProps(i).InitCond.Quad = Data.Agents.Quad.States(1:6,1);
for j = 1:2
    AllProps(i).InitCond.(['Target',num2str(j)]) = Data.Agents.Target(j).States(1:6,1);
end
AllProps(i).InitCond.DropSite = Environment.Geometry.DropLocation;

%% FINITE STATE TIMES

% Average time in state
StateTimes = {};
j = 0;
t0 = Data.Time(1);
State = Data.Agents.Quad(1).Mode{1};
for k = 2:length(Data.Time)
    
    if ~strcmp(State,Data.Agents.Quad(1).Mode{k}) || k == length(Data.Time)
        j = j + 1;
        StateTimes{j,1} = State;
        StateTimes{j,2} = Data.Time(k) - t0;
        StateTimes{j,3} = t0;
        t0 = Data.Time(k);
        State = Data.Agents.Quad(1).Mode{k};
    end
    
end

% Find duplicates in StateTimes and take average before adding to
% properties
StateTimesSorted = StateTimes(1,:);
StateTimesSorted{3} = 1;
j = 1;
for k = 2:size(StateTimes,1)
    if ~ismember(StateTimes{k,1},StateTimesSorted(:,1))
        j = j + 1;
        StateTimesSorted{j,1} = StateTimes{k,1};
        StateTimesSorted{j,2} = StateTimes{k,2};
        StateTimesSorted{j,3} = 1;
    else
        j2 = find(ismember(StateTimesSorted(:,1),StateTimes{k,1}));
        StateTimesSorted{j2,2} = StateTimesSorted{j2,2} + StateTimes{k,2};
        StateTimesSorted{j2,3} = StateTimesSorted{j2,3} + 1;
    end
end

% Save average times
for k = 1:size(StateTimesSorted,1)
    StateTimesAveraged{k,1} = StateTimesSorted{k,1};
    StateTimesAveraged{k,2} = StateTimesSorted{k,2}/StateTimesSorted{k,3};
end

% Save to structure
AllProps(i).StateTimes = StateTimes;
AllProps(i).AvgStateTimes = StateTimesAveraged;
AllProps(i).TotalStateTimes = StateTimesSorted;

%% FINITE STATE DISTANCE

% Get total distance covered during simulation
dr = diff(Data.Agents.Quad.States(1:3,:),1,2);
D = [0 sqrt(dr(1,:).^2 + dr(2,:).^2 + dr(3,:).^2)];

% Loop through identified states
for k = 1:size(StateTimesSorted,1)
    
    % Identify indices in mode array
    ind = find(ismember(Data.Agents.Quad.Mode,StateTimesSorted{k,1}));
    
    % Total distance in mode
    StateDistanceSorted{k,1} = StateTimesSorted{k,1};
    StateDistanceSorted{k,2} = sum(D(ind));
    StateDistanceSorted{k,3} = StateTimesSorted{k,3};
    
    % Average distance in mode
    StateDistanceAveraged{k,1} = StateTimesSorted{k,1};
    StateDistanceAveraged{k,2} = sum(D(ind))/StateTimesSorted{k,3};
    
end

% Save to structure
AllProps(i).TotalStateDistance = StateDistanceSorted;
AllProps(i).AvgStateDistance = StateDistanceAveraged;

%% FINITE STATE TRANSITIONS

% Loop through each state in current run
for k = 1:size(StateTimes,1)-1
    
    % For each state, identify succeeding state
    StateTransitionsAll{k,1} = StateTimes{k,1};
    StateTransitionsAll{k,2} = StateTimes{k+1,1};
    
end

% Loop through each state pair and identify no. of occurrences
StateTransitions = {};
j = 0;
for k = 1:size(StateTransitionsAll,1)
    
    % Current pair
    CurrentPair = StateTransitionsAll(k,:);
    
    % Check no. of occurrences
    Occ = sum(all(ismember(StateTransitionsAll,CurrentPair),2));
    
    % Save to output array if not already present
    if isempty(StateTransitions) || ~any(all(ismember(StateTransitions(:,1:2),CurrentPair),2))
        j = j + 1;
        StateTransitions(j,1) = CurrentPair(1);
        StateTransitions(j,2) = CurrentPair(2);
        StateTransitions{j,3} = Occ;
    end
    
end

% Save to structure
AllProps(i).StateTransitions = StateTransitions;

%% PROBABILITIES

% Check occurrence of faults from state transitions
SystemsFault = 0;
ActuatorFault = 0;
GrabberFault = 0;

j = 0;
k2 = 0;
Faults = {};
for k = 1:length(Data.Time)
    
    % Systems fault depends on state following Initialise
    if ismember('Initialise',Data.Agents.Quad(1).Mode(k))...
            && ismember('Search',Data.Agents.Quad(1).Mode(k+1))
        j = j + 1;
        SystemsFault(j) = 0;
    elseif ismember('Initialise',Data.Agents.Quad(1).Mode(k))...
            && ismember('Land',Data.Agents.Quad(1).Mode(k+1))
        j = j + 1;
        SystemsFault(j) = 1;
        k2 = k2 + 1;
        Faults{k2,1} = 'System';
        Faults{k2,2} = Data.Time(k);
    end
    
    % Actuator fault depends on occurrence of Emergency Land
    if ismember('Emergency land',Data.Agents.Quad(1).Mode(k))
        ActuatorFault = 1;
        k2 = k2 + 1;
        Faults{k2,1} = 'Actuator';
        Faults{k2,2} = Data.Time(k);
    end
    
    % Grabber fault depends on occurrence of Reacquire Target
    if ismember('Reacquire target',Data.Agents.Quad(1).Mode(k))
        GrabberFault = 1;
        k2 = k2 + 1;
        Faults{k2,1} = 'Grabber';
        Faults{k2,2} = Data.Time(k);
    end
    
end

% Save to structure
AllProps(i).SystemsFault = SystemsFault;
AllProps(i).ActuatorFault = ActuatorFault;
AllProps(i).GrabberFault = GrabberFault;
AllProps(i).Faults = Faults;

%% SAVE NEW DATA TO SAVE STRUCTURE
% if ~exist(Props)
%     Props = [];
% end
save(SaveFile,'AllProps')