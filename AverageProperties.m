% Get probabilities and averages of continuous-time model from collected
% data

function AverageProperties(SaveFile)

% close all
% clearvars -except h GlobalTime Count Run Runs SaveFile Data Agents Environment Sim
% clc

fprintf('Averaging properties for all runs...\n\n')

% Load dataset
load(SaveFile)

%% BASIC PROPERTIES

% Numbers of runs averaged
Props.NumRuns = length(AllProps);

% Average mission time
Props.AvgMissionTime = sum(cat(1,AllProps.MissionTime))/length(AllProps);

% Mission success probability
Props.MissionSuccess = sum(cat(1,AllProps.MissionOutcome))/length(AllProps);

%% RUN LOOP

% Initialise arrays
TotalStateTimes = {};
TotalStateDistances = {};
TotalStateTransitions = {};
TotalSystemsFault = [0 0];
TotalActuatorFault = [0 0];
TotalGrabberFault = [0 0];

% Loop runs
for i = 1:length(AllProps)
    
    % STATE-SPECIFIC PROPERTIES -------------------------------------------
    
    % Loop states
    for j = 1:size(AllProps(i).AvgStateTimes,1)
        
        % Current state
%         CurrentState = AllProps(i).AvgStateTimes{j,1};
        CurrentState = AllProps(i).TotalStateTimes{j,1};
        
        % Current time
%         CurrentTime = AllProps(i).AvgStateTimes{j,2};
        CurrentTime = AllProps(i).TotalStateTimes{j,2};
        
        % Current total
        CurrentTotal = AllProps(i).TotalStateTimes{j,3};
        
        % Current distance
%         CurrentDistance = AllProps(i).AvgStateDistance{j,2};
        CurrentDistance = AllProps(i).TotalStateDistance{j,2};
        
        % Check if current state already exists in total matrix
        if isempty(TotalStateTimes) || ~ismember(CurrentState,TotalStateTimes(:,1))
            
            % If state doesn't exist, add it and initialise with first
            % total
            
            TotalStateTimes{end+1,1} = CurrentState;
            TotalStateTimes{end,2} = CurrentTime;
%             TotalStateTimes{end,3} = 1;
            TotalStateTimes{end,3} = CurrentTotal;
            
            TotalStateDistances{end+1,1} = CurrentState;
            TotalStateDistances{end,2} = CurrentDistance;
%             TotalStateDistances{end,3} = 1;
            TotalStateDistances{end,3} = CurrentTotal;
            
        else
            
            % If state exists, add current value to total
            
            % Get index of current state in array
            k = find(ismember(TotalStateTimes(:,1),CurrentState));
            
            % Add to counter
            TotalStateTimes{k,2} = TotalStateTimes{k,2} + CurrentTime;
%             TotalStateTimes{k,3} = TotalStateTimes{k,3} + 1;
            TotalStateTimes{k,3} = TotalStateTimes{k,3} + CurrentTotal;
            
            TotalStateDistances{k,2} = TotalStateDistances{k,2} + CurrentDistance;
%             TotalStateDistances{k,3} = TotalStateDistances{k,3} + 1;
            TotalStateDistances{k,3} = TotalStateDistances{k,3} + CurrentTotal;
            
        end
        
    end
    
    % STATE TRANSITIONS ---------------------------------------------------
    
    for j = 1:size(AllProps(i).StateTransitions,1)
        
        % Current pair
        CurrentPair = AllProps(i).StateTransitions(j,:);
        
        % Check if current pair already exists in total matrix
        if isempty(TotalStateTransitions) || ~any(all(ismember(TotalStateTransitions(:,1:2),CurrentPair(1:2)),2))
            
            % If state pair doesn't exist, add it and initialise it with
            % first total
            
            TotalStateTransitions(end+1,:) = CurrentPair;
            
        else
            
            % If state pair exists, add current value to total
            k = find(all(ismember(TotalStateTransitions(:,1:2),CurrentPair(1:2)),2));
            
            % Add to counter
            TotalStateTransitions{k,3} = TotalStateTransitions{k,3} + CurrentPair{3};
            
        end
        
    end
    
    % AVERAGE OCCURRENCE OF FAULTS ----------------------------------------
    
    % Systems fault
    TotalSystemsFault = TotalSystemsFault...
        + [sum(AllProps(i).SystemsFault) length(AllProps(i).SystemsFault)];
    
    % Actuator fault
    TotalActuatorFault = TotalActuatorFault + [AllProps(i).ActuatorFault 1];
    
    % Grabber fault
    TotalGrabberFault = TotalGrabberFault + [AllProps(i).GrabberFault 1];
    
end

%% AVERAGE TOTALLED DATA

% Get average of each state, ignoring cases where the state did not occur
for j = 1:size(TotalStateTimes,1)
    
    Props.AvgStateTimes{j,1} = TotalStateTimes{j,1};
    Props.AvgStateTimes{j,2} = TotalStateTimes{j,2}/TotalStateTimes{j,3};
    
    Props.AvgStateDistances{j,1} = TotalStateDistances{j,1};
    Props.AvgStateDistances{j,2} = TotalStateDistances{j,2}/TotalStateDistances{j,3};
    
end

% Get average occurrences of state transitions
ProbStateTransitions = TotalStateTransitions;
for j = 1:size(TotalStateTransitions,1)
    
    NextStates = TotalStateTransitions(ismember(TotalStateTransitions(:,1),TotalStateTransitions(j,1)),2);
    TransOcc = TotalStateTransitions(ismember(TotalStateTransitions(:,1),TotalStateTransitions(j,1)),3);
    TransOcc = sum(cat(1,TransOcc{:}));
    ProbStateTransitions{j,3} = TotalStateTransitions{j,3}/sum(TransOcc);
    
end

% Sort state transitions
Props.ProbStateTransitions = sortrows(ProbStateTransitions,1);

% Average occurrence of systems fault
Props.ProbSystemsFault = TotalSystemsFault(1)/TotalSystemsFault(2);

% Average occurence of actuator fault
Props.ProbActuatorFault = TotalActuatorFault(1)/TotalActuatorFault(2);

% Average occurence of actuator fault
Props.ProbGrabberFault = TotalGrabberFault(1)/TotalGrabberFault(2);

%% SAVE AVERAGED DATA TO SAVE STRUCTURE

save(SaveFile,'Props','AllProps')



