function [Yd,Exitflag] = SearchPattern(obj,Y,t)

persistent lastWP;
persistent searchMove;

Exitflag = 0;

% Initialize waypoints
if isempty(lastWP) || strcmp(obj.Mode,'Initialise')
    searchMove = 0;
    lastWP = 100;
    Yd = 0;
    ExitFlag = 0;
% Check if at gridpoint
elseif norm(Y([1:3,6])-obj.Waypoints(obj.WP,1:4)') < 1e-2
    
    x = round(obj.States(1)/obj.CellSize);
    y = round(obj.States(2)/obj.CellSize);
    
    [x,y] = adjustCoords(obj, x, y);
    fprintf('Time %4.2f Searching: (%d,%d)\n',obj.Time,x,y)
    % Check battery
    if obj.BatteryLevel <= 2*(abs(x)+abs(y)) && ~obj.BatteryWarning && norm(obj.States(1:2))
        fprintf('Time %4.2f s: BATTERY WARNING\n',obj.Time)
        obj.Cint = 0;
        obj.BatteryWarning = 1;
    % If at lastWP we're in the middle of a search
    elseif obj.WP == lastWP
        %fprintf('Time %4.2f s: At:%d, Navigating from:%d, SearchMove:%d \n',obj.Time,obj.WP,lastWP,searchMove)
        
        % Track waypoints
        switch obj.WP
            case {1, 12, 23, 34}
                searchMove = 1;
            case {10, 19, 28, 37}
                searchMove = 10;
            case {68, 79, 90, 100}
                searchMove = -1;
            case {64, 73, 82, 91}
                searchMove = -10;
            case {57}
                if ~obj.Decisions.Mode
                    obj.MissionFailed = 1;
                    obj.FailureType = "Searched full grid: still missing objects.";
                end
        end
        
        obj.WP = obj.WP + searchMove;
        lastWP = obj.WP;
    else
        % move to WP in search

        if mod(obj.WP-1,10) < mod(lastWP-1,10)
            returnMove = 1;
        elseif mod(obj.WP-1,10) > mod(lastWP-1,10)
            returnMove = -1;
        else
            if obj.WP < lastWP
                returnMove = 10;
            else
                returnMove = -10;
            end
        end
        %fprintf('Time %4.2f s: At:%d, Navigating to:%d, ReturnMove:%d, SearchMove:%d \n',obj.Time,obj.WP,lastWP+searchMove,returnMove,searchMove)
        obj.WP = obj.WP + returnMove;
    end
%    else
        % Something has gone wrong, reset search pattern.
%        lastWP = 1;
%        obj.Mode = 'Return to base';
%    end
    if ~obj.BatteryWarning
        obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate;
    end
end

if obj.WP > obj.MaxWP
    obj.WP = obj.MaxWP;
    Exitflag = 1;
else
    Exitflag = 0;
end
Yd = obj.Waypoints(obj.WP,:)';