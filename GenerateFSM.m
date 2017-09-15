% Generate finite state machine from Monte Carlo data

close all
clear
clc

% Plot method
% 1 - circle
Method = 1;

% Load data
load('ModelPropsTestRun-Use.mat')

% Identify all states
StateOrder = {'Idle'
          'Take-off'
          'Initialise'
          'Search'
          'Identify'
          'Reacquire target'
          'Hover above target'
          'Descend to grab'
          'Grab'
          'Ascend'
          'Transport'
          'Descend to drop'
          'Drop'
          'Return to search'
          'Return to base'
          'Land'
          'Emergency land'};

% State of interest
iState = 'Return to search';
% iState = StateOrder(7);

% Catalogue states in desired order
j = 0;
for i = 1:length(StateOrder)
    
    % Current state
    State = StateOrder(i);
    
    % Ensure state is in data
    if any(ismember(Props.AvgStateTimes(:,1),State))
        
        j = j + 1;
        States(j,1) = State;
        
    end
    
end

NumStates = length(States);
Transitions = Props.ProbStateTransitions;
NumTransitions = size(Transitions,1);

figure('Color','w','OuterPosition',CentreFig(1200,800))
hold on, axis equal, axis off

% Plot states with desired method
switch Method
    case 1
        
        % Settings
        r = [1.4 1];  % Circle radii
        dy = 0.1;  % State radii
        N = 24; % State circle sides
        
        % Azimuth parameter
        p = (0:1/NumStates:1-1/NumStates)*2*pi;
            
        % Plot states
        for i = 1:NumStates
            
            % Length of text
            dx = 0.013*length(States{i});
            dx(dx<0.1) = 0.1;
            
            C(i,:) = [-r(1)*cos(p(i))
                       r(2)*sin(p(i))]';
            V = [dx*cos((1/N:1/N:1)*2*pi) + C(i,1)
                 dy*sin((1/N:1/N:1)*2*pi) + C(i,2)
                 0*(1/N:1/N:1)]';
            F = 1:N;
            
            patch('Faces',F,'Vertices',V,'FaceColor','w','EdgeColor',0.4*[1 1 1])
            text(C(i,1),C(i,2),States{i},'HorizontalAlignment','center')
             
        end
        
        % Plot state transitions
        for i = 1:NumTransitions
            
            % Get start and end points
            if strcmp(Transitions(i,1),iState) || strcmp(iState,'all')
                t1 = C(ismember(States,Transitions(i,1)),:);
                t2 = C(ismember(States,Transitions(i,2)),:);
                T = [t1; t2]';
                
                % Point centroid
                Tc = (t1 + t2)/2;
                
                % Plot line
                plot3(T(1,:),T(2,:),[-1 -1],'-','Color',0.4*[1 1 1])
                
                % Plot probability at centroid
                Prob = Transitions{i,3};
                text(Tc(1),Tc(2),1,num2str(Prob,'%.5f'))
                
            end
            
        end
        
end



