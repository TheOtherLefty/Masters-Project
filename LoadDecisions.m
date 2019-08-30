function [stateTable, transitionTable] = LoadDecisions(dataName)

% close all
% clear
% clc
% 

%% Load data

% Load states
stateDataRaw = importdata(['ControllerV2/',dataName,'.sta']);

% Load transitions
transDataFile = fopen(['ControllerV2/',dataName,'.tra']);
transDataRaw = textscan(transDataFile, '%s %s %s %s %s');
fclose(transDataFile);

%% Format states

% Remove brackets and split at comma
for i = 1:size(stateDataRaw,1)
    
    % Find brackets
    startBr = strfind(stateDataRaw{i},'(');
    endBr = strfind(stateDataRaw{i},')');
    
    % First element in row is state
    if i == 1
        state = 'state';
    else
        state = stateDataRaw{i}(1:startBr-2);
    end
    
    % Keep only characters in between
    row = stateDataRaw{i}(startBr+1:endBr-1);
    
    % Split string at commas
    row = strsplit(row,',');
    
    % Add to new array
    stateData(i,:) = [state row];
    
end

% Split data
stateHeaders = stateData(1,:);
stateValues = zeros(size(stateData,1), size(stateData,2));
stateValues = str2double(stateData(2:end,:));

stateTableOld = array2table(stateValues, 'VariableNames', stateHeaders);

stateTable = array2table(stateValues(:,1:5), 'VariableNames', stateHeaders(1:5));
gpsStates = stateValues(:,6:end-1);
stateTable.gps = gpsStates;
stateTable.b = stateValues(:,end);

%% Format transitions

% Fix things
for i = 2:length(transDataRaw{1})
    currentState(i,1) = str2num(transDataRaw{1}{i});
    something(i,1) = str2num(transDataRaw{2}{i});
    nextState(i,1) = str2num(transDataRaw{3}{i});
    probability(i,1) = str2num(transDataRaw{4}{i});
    direction(i,1) = transDataRaw{5}(i);
    
end

transitionTable = table(currentState,nextState,probability,direction,...
    'VariableNames', {'state','next','prob','dir'});


