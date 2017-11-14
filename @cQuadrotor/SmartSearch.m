function [Yd, ExitFlag] = SmartSearch(obj, Y, t)

% If at grid point, work out where to go
if norm(Y([1:3,6])-obj.Waypoints(obj.WP,1:4)') < 1e-2
    
    % Current position and grid point
    x = round(Y(1)/obj.CellSize);
    y = round(Y(2)/obj.CellSize);
    gp = y*obj.GridSize(1) + x;
    
    % Num of targets to find
    objs = obj.NumTargets - obj.TargetCount;
    
    % Find current state in table
    tol = 1e-6;
    cond{1} = abs(obj.Decisions.States.posx - x) < tol;
    cond{2} = abs(obj.Decisions.States.posy - y) < tol;
    cond{3} = abs(obj.Decisions.States.objs - objs) < tol;
    cond{4} = abs(sum(obj.Decisions.States.gps - obj.Decisions.Gridpoints,2)) < tol;
    cond{5} = abs(obj.Decisions.States.b - obj.BatteryLevel) < tol;
    
    % Reset battery if at (0, 0) and it's empty
    if abs(x) < tol && abs(y) < tol && abs(obj.BatteryLevel) < tol
        obj.BatteryLevel = obj.BatteryMax - obj.BatteryLossRate;
    end
    
    % Checks
%     check12 = find(cond1 & cond2)
%     check3 = find(cond3)
%     check4 = find(cond4)
    
    states_all = obj.Decisions.States.state(cond{1} & cond{2} & cond{3} & cond{4} & cond{5});
    
    % Limit to first solution only
    try
        state = states_all(1);
    catch ME
        states_all
        for i = 1:length(cond)
            for j = 1:length(cond)
                BoolTable(i,j) = any(cond{i} & cond{j});
            end
        end
        fprintf('Previous state: %d\n', obj.Decisions.CurrentState)
        fprintf('Current state values: posx: %d, posy: %d, objs: %d, b: %d\n', x, y, objs, obj.BatteryLevel)
        fprintf('    gps = [ '), fprintf('%d ', obj.Decisions.Gridpoints), fprintf(']\n\n')
        fprintf('Condition table:\n')
        fprintf('   x   y objs gps  b \n')
        disp(BoolTable)
    end
    
    obj.Decisions.CurrentState = state;
    
    % Get direction to move and predicted next states
    dir = obj.Decisions.Transitions.dir(obj.Decisions.Transitions.state == state);
    nextState = obj.Decisions.Transitions.next(obj.Decisions.Transitions.state == state);
    
    % Get first direction only, should all be the same
    dir = dir{1};
    
    fprintf('    Current state: %d | Direction: %s | Next state(s):',state, dir)
    fprintf(' %d',nextState)
    fprintf('\n')
    
    % Set current grid point as searched
    obj.Decisions.Gridpoints(gp+1) = 0;
    
    % Get next grid point to move to
    if strcmp(dir,'east')
        x = x + 1;
    elseif strcmp(dir,'west')
        x = x - 1;
    elseif strcmp(dir,'north')
        y = y + 1;
    elseif strcmp(dir, 'south')
        y = y - 1;
    else
        error(['Something is wrong, dir is ',dir])
    end
    
%     newgp = y*obj.GridSize(1) + x
    obj.WP = y*obj.GridSize(1) + x + 1;
    obj.WPcount = obj.WPcount + 1;
    obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate;
    
%     Yd = obj.Waypoints(obj.WP,:)'
    
%     pause
    
end

Yd = obj.Waypoints(obj.WP,:)';
ExitFlag = 0;

