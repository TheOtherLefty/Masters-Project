function [Yd] = SmartSearch(obj, Y, t)

% Ensures we only make decisions when at a gridpoint.
if norm(Y(1:3)-obj.Waypoints(obj.WP,1:3)') < 1e-2
    
    % Current position and grid point
    [x,y] = adjustCoords(obj,round(Y(1)/obj.CellSize),round(Y(2)/obj.CellSize));
    
    gp = nearestBaseWP(obj) + (y * 10) + x;
    tol = 1e-6;
    
    % Num of targets to find
    objs = obj.NumTargets - obj.TargetCount;
    
    if obj.Decisions.Mode
        FoundObj = 1;
    else
        FoundObj = 0;
    end
    
    %------------------------------State Identification--------------------
    % Note: SearchStatus is 0 initially, we deal with found object states
    % later.
    SearchStatus = 0;
    
    [state,flag] = IdentifyState(obj, Y, SearchStatus);
    if flag && atBase(obj)
        % Very specific problem when transitioning controller files, lazy
        % solution but it works if the battery level is changed temporarily
        % to 34 to find the state before changing back.
        obj.BatteryLevel = obj.MaxBattery;
        [state,flag] = IdentifyState(obj, Y, SearchStatus);
        
        % If there's still an error then something really has gone wrong.
        if flag
            fprintf('Error: Previous state: %d\n', obj.Decisions.CurrentState)
            fprintf('Current state values: posx: %d, posy: %d, objs: %d, b: %d\n', x, y, objs, obj.BatteryLevel);

            fprintf('    gps = [ '), fprintf('%d ', obj.Decisions.Gridpoints), fprintf(']\n\n')
            error(['Unrecognized State'])
        end
    elseif flag
        % If all the gridpoints have been searched the search has failed
        % and the quad will return to base.
        if sum(obj.Decisions.Gridpoints(:)==1)<(obj.NumTargets-obj.TargetCount)
            obj.MissionFailed = 1;
            obj.FailureType = "Searched full grid: still missing objects.";
        else
            fprintf('Error: Previous state: %d\n', obj.Decisions.CurrentState)
            fprintf('Current state values: posx: %d, posy: %d, objs: %d, b: %d\n', x, y, objs, obj.BatteryLevel);

            fprintf('    gps = [ '), fprintf('%d ', obj.Decisions.Gridpoints), fprintf(']\n\n')
            error(['Unrecognized State'])
        end
    end
    
    if obj.MissionFailed
        obj.WP = nearestBaseWP(obj);
    else
        obj.Decisions.CurrentState = state;

        % Set current grid point as searched
        obj.Decisions.Gridpoints(gp) = 0;

        % Reset battery if at a base gridpoint
        if atBase(obj)
            obj.BatteryUsage = obj.BatteryUsage + (obj.MaxBattery - obj.BatteryLevel);
            obj.BatteryLevel = obj.MaxBattery - obj.BatteryLossRate;
        else
            obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate;
        end

        %------------------------------Decision Making-------------------------
        if ~FoundObj
            dir = obj.Decisions.Transitions.dir(obj.Decisions.Transitions.state == state);
            nextState = obj.Decisions.Transitions.next(obj.Decisions.Transitions.state == state);

            % There will be multiple transitions out of the state, but
            % fortunately they're all go in the the same direction
            dir = dir{1};
        else
            % Set directive and find next state, now looking for found object
            % states
            SearchStatus = 1;

            dir = 'Identify object';
            [nextState, flag] = IdentifyState(obj, Y, SearchStatus); % get found object state
            if flag
                fprintf('Error: Previous state: %d\n', obj.Decisions.CurrentState)
                fprintf('Current state values: posx: %d, posy: %d, objs: %d, b: %d\n', x, y, objs, obj.BatteryLevel)
                fprintf('    gps = [ '), fprintf('%d ', obj.Decisions.Gridpoints), fprintf(']\n\n')
                error(['Unrecognized State'])
            end
        end

        fprintf('    Current state: %d | Directive: %s | Next state(s):',state, dir)
        fprintf(' %d',nextState)
        fprintf('\n')

        % If found object, update state and return to base
        % Else update x and y to reflect the impact of our chosen move.
        if strcmp(dir,'Identify object')
            obj.Decisions.CurrentState = nextState;
            nextState = obj.Decisions.Transitions.next(obj.Decisions.Transitions.state == nextState);
            fprintf('    Current state: %d | Directive: Deposit at base | Next state(s):',state)
            fprintf(' %d',nextState)
            fprintf('\n')

            [x,y] = adjustCoords(obj,round(Y(1)/obj.CellSize),round(Y(2)/obj.CellSize));
            % adjust x and y coordinate values depending on quadrant.

            obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate*(abs(x) + abs(y));
            fprintf('Transporting decreased battery by %d ; Coordinates = (%d,%d); New battery value: %d \n', 2*(x + y), x, y, obj.BatteryLevel)
            % If no direction is returned this means we're at the end of the controller file, indicating quadrant has been
            % searched fully.
            
            % I hate this but using only using all of these conditions does
            % it always transition between files 
        elseif (strcmp(dir, '') | (state == nextState)) | (all(~translateGPS(obj)) & atBase(obj))
                obj.SearchedQuadrants = obj.SearchedQuadrants + 1;

                % Switches to the next controller file and returns the base
                % gridpoint for the quadrant that that controller is for.
                obj.WP = transitionController(obj);
        else
            % Converts direction based on the quadrant quad is currently in.
            dir = translateDirection(obj,dir);
            % Get next grid point to move to
            if strcmp(dir,'east')
                x = x + 1;
            elseif strcmp(dir,'west')
                x = x - 1;
            elseif strcmp(dir,'north')
                y = y + 1;
            elseif strcmp(dir,'south')
                y = y - 1;
            end

            % Sets waypoint to new x and y locations.
            obj.WP = nearestBaseWP(obj) + (y * 10 + x);
            obj.WPcount = obj.WPcount + 1;
        end
    end
end

Yd = obj.Waypoints(obj.WP,:)';

    