function [Yd,Exitflag] = SearchPattern(obj,Y,t)

persistent lastWP;

Exitflag = 0;

% Initialize waypoints
if isempty(lastWP) || strcmp(obj.Mode,'Initialise')
    lastWP = 56; %56 is mid waypoint in 10/10 grid; this is where base is
    Yd = 0;
    ExitFlag = 0;
% Check if at gridpoint
elseif norm(Y([1:3,6])-obj.Waypoints(obj.WP,1:4)') < 1e-2
    
    x = round(obj.States(1)/obj.CellSize);
    y = round(obj.States(2)/obj.CellSize);
    
    fprintf('Time %4.2f Searching: (%d,%d)\n',obj.Time,x,y)
    
    % Check battery
    if obj.BatteryLevel <= 2*(x+y) && ~obj.BatteryWarning && norm(obj.States(1:2))
        fprintf('Time %4.2f s: BATTERY WARNING\n',obj.Time)
        obj.Cint = 0;
        obj.BatteryWarning = 1;
    % If at lastWP we're in the middle of a search
    elseif obj.WP == lastWP
        % Track waypoints
        switch lastWP
            case {56}
                obj.WP = obj.WP -10;
            case {46}
                obj.WP = obj.WP +10;
            case 25
                if ~obj.Decisions.Mode
                    obj.MissionFailed = 1;
                    obj.FailureType = "Searched full grid: still missing objects.";
                end
        end
        lastWP = obj.WP;
        
    else
        
        if obj.WP == 56
            obj.WP = 46;
        else
            obj.WP = 56;
        end
        lastWP = obj.WP;
    end
%    else
        % Something has gone wrong, reset search pattern.
%        lastWP = 1;
%        obj.Mode = 'Return to base';
%    end
    
    obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate;
    
end

if obj.WP > obj.MaxWP
    obj.WP = obj.MaxWP;
    Exitflag = 1;
else
    Exitflag = 0;
end
Yd = obj.Waypoints(obj.WP,:)';