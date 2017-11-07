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
    cond1 = abs(obj.Decisions.States.posx - x) < tol;
    cond2 = abs(obj.Decisions.States.posy - y) < tol;
    cond3 = abs(obj.Decisions.States.objs - objs) < tol;
    cond4 = abs(sum(obj.Decisions.States.gps - obj.Decisions.Gridpoints,2)) < tol;
    % cond5 = Battery
    
    % Checks
%     check12 = find(cond1 & cond2)
%     check3 = find(cond3)
%     check4 = find(cond4)
    
    state = obj.Decisions.States.state(cond1 & cond2 & cond3 & cond4);
    
    % Limit to first solution only
    state = state(1);
    
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
    
%     Yd = obj.Waypoints(obj.WP,:)'
    
%     pause
    
end

Yd = obj.Waypoints(obj.WP,:)';
ExitFlag = 0;

