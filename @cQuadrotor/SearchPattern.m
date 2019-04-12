function [Yd,Exitflag] = SearchPattern(obj,Y,t)

persistent lastWP;

Exitflag = 0;

x = round(obj.States(1)/obj.CellSize);
y = round(obj.States(2)/obj.CellSize);

% Initialize waypoints
if isempty(lastWP) || obj.Time < 1
    lastWP = 1;
% Check if at gridpoint
elseif norm(Y([1:3,6])-obj.Waypoints(obj.WP,1:4)') < 1e-2
    % Check battery
    if obj.BatteryLevel <= 2*(x+y) && ~obj.BatteryWarning && norm(obj.States(1:2))
        fprintf('Time %4.2f s: BATTERY WARNING\n',obj.Time)
        obj.Cint = 0;
        obj.BatteryWarning = 1;
    % If at lastWP we're in the middle of a search
    elseif obj.WP == lastWP
        % Track waypoints
        switch lastWP
            case {1, 2, 3, 4, 11, 12, 13, 14, 21, 22, 23, 24}
                obj.WP = obj.WP + 1;
            case {7, 8, 9, 10, 17, 18, 19, 20}
                obj.WP = obj.WP - 1;
            case  {5, 6, 15, 16}
                obj.WP = obj.WP + 5;
            case 25
                if ~obj.Decisions.Mode
                    obj.MissionFailed = 1;
                    obj.FailureType = "Searched full grid: still missing objects.";
                end
        end
        lastWP = obj.WP;
    % Check if we're in the right x column
    elseif mod(obj.WP-1,5) < mod(lastWP-1,5)
        obj.WP = obj.WP+1;
    % Check if we're in the right y column
    elseif floor(obj.WP/5) < floor(lastWP/5)
        obj.WP = obj.WP+5;
    else
        lastWP
        obj.WP
        error(['Cant find lastWP'])
    end
    obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate;
end

if obj.WP > obj.MaxWP
    obj.WP = obj.MaxWP;
    Exitflag = 1;
else
    Exitflag = 0;
end

Yd = obj.Waypoints(obj.WP,:)';