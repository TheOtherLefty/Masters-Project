function State = StateMachine(obj,State)

% Check for mode changes
if ~strcmp(State,obj.PrevMode)
    obj.ModeEntryTime = obj.Time;
    obj.ModeTransition = 'Entry';
end
obj.PrevMode = State;
TargetFound = 0;

if isempty(obj.Decisions)
    obj.obj.SearchType = "Pattern";
end

switch State
    
    case 'Idle'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            
            fprintf('Time %4.2f s: Agent idle\n',obj.Time)
            if obj.MissionComplete
                obj.BatteryUsage = obj.BatteryUsage + (obj.MaxBattery - obj.BatteryLevel);
                fprintf('    MISSION COMPLETE\n')
            elseif obj.MissionFailed
                fprintf('    MISSION FAILED\n')
            end
        end
        
        % Update controller
        obj.Inputs = [0 0 0 0]';
        obj.epos_int = [0 0 0]';
        
        % State transition conditions
        if ~obj.MissionComplete && ~obj.MissionFailed
            State = 'Take-off';
        elseif (obj.MissionFailed || obj.MissionComplete) && obj.Time - obj.ModeEntryTime > 2
            obj.MissionEnd = 1;
        end
        
    case 'Take-off'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            obj.BatteryRate = obj.LossRate;
            fprintf('Time %4.2f s: Taking off\n',obj.Time)
        end
        
        obj.InitWaypoints;
        
        % Update navigation
        wp = obj.Waypoints(nearestBaseWP(obj),:);
        Commands = [wp(1:2) -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % State transition conditions
        if obj.QuadInitialized == 0
            State = 'Initialise';
        elseif norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-1
            obj.MissionComplete = 0;
            State = 'Search';
        end
        
    case 'Initialise'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Initialising\n',obj.Time)
            obj.BatteryLevel = obj.MaxBattery;
        end
        
        % Initialize search algorithm incase of requirement
        [~,~] = obj.SearchPattern(obj.States,obj.Time-obj.ModeEntryTime);
        
        % Update navigation
        wp = obj.Waypoints(nearestBaseWP(obj),:);
        Commands = [wp(1:2) -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        obj.SystemsOkay = rand < (1 - obj.P_SystemsFault);
        
        obj.QuadInitialized = 1;
        
        % State transition conditions
        if obj.SystemsOkay && obj.Time - obj.ModeEntryTime > 0.5
            fprintf('    Systems okay, continuing mission\n')
            obj.SystemsWarning = 0;
            State = 'Search';
        elseif ~obj.SystemsOkay && obj.Time - obj.ModeEntryTime > 0.5
            fprintf('    Systems not okay, landing aircraft\n')
            obj.SystemsWarning = 1;
            State = 'Land';
        end
        
    case 'Search'
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.InitWaypoints;
            obj.ModeTransition = 'Active';
            obj.VelLimit = 2;
            fprintf('Time %4.2f s: Searching\n',obj.Time)
            TargetFound = 0; %if we've entered this state then we know no object has been found
        end

        % Update camera
        [Coordinates,DropSite] = obj.CameraModel(obj.States, obj.Time);
        
        x = obj.States(1)/obj.CellSize;
        y = obj.States(2)/obj.CellSize;
        
        %Update object tracking only when not above base
        if x >= 0.9 || y >= 0.9 || y <= -1.9 || x <= -1.9
            [~,TargetFound,~] = obj.ObjectTracking(Coordinates,DropSite);
        end
        
        % Update state if target is found at current waypoint
        if obj.Decisions.Mode && norm(obj.States(1:3)-obj.Waypoints(obj.WP,1:3)') < 1e-2
            
            % Movements of the camera sometimes result in the quad
            % glimpsing an object in an adjacent gp while it is in/moving to a previously searched location; this
            % checks if it is possible to find an object at the current location.
            if strcmp(obj.SearchType,"Smart")
                
                [x,y] = adjustCoords(obj,round(x),round(y));
                gp = nearestBaseWP(obj) + (y * 10 + x);
                
                if gp
                    % Make adjustments to check for found state
                    obj.Decisions.Gridpoints(gp) = 0;
                    obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate;
                    
                    % Check state
                    [~, falsePositive] = IdentifyState(obj, obj.States, obj.Decisions.Mode);
                    
                    % Undo changes
                    obj.Decisions.Gridpoints(gp) = 1;
                    obj.BatteryLevel = obj.BatteryLevel + obj.BatteryLossRate;
                else
                    % Objects can't be found in already searched locations
                    % so this must be a false positive
                    falsePositive = 1;
                end
                
                if falsePositive
                    TargetFound = 0;
                    obj.Decisions.Mode = 0;
                    fprintf('False Positive\n')
                else
                    fprintf('    Identifying found target\n')
                    State = 'Identify';
                    obj.VelLimit = obj.VelLimitSaved;
                end
            elseif strcmp(obj.SearchType,"Pattern")
                    % Pattern search is unaffected by false positives
                    fprintf('    Identifying found target\n')
                    State = 'Identify';
                    obj.VelLimit = obj.VelLimitSaved;
            end
        elseif ~obj.Decisions.Mode && TargetFound
            
            fprintf('    Target found, waiting to identify\n')
            obj.Decisions.Mode = 1;
        end
        
        if strcmp(obj.SearchType,"Smart")
            try
                [Commands] = obj.SmartSearch(obj.States,obj.Time-obj.ModeEntryTime);
            catch
                fprintf("SmartSearch encountered an error: Enabling Pattern search.\n")
                obj.SearchType = "Pattern";
                State = 'Return to base' ;           
            end
        end
        if strcmp(obj.SearchType,"Pattern")
            [Commands, ~] = obj.SearchPattern(obj.States,obj.Time-obj.ModeEntryTime);
            if obj.BatteryWarning
                State = 'Return to base';
            end
        end
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);

        if obj.MissionFailed
            fprintf('     All grid-points searched, returning to base\n')
            State = 'Return to base';
            obj.VelLimit = obj.VelLimitSaved;
        else
            % Update controller
            [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
                obj.Controller(obj.StatesEst,Commands,obj.Time,0);

            % Exit conditions
            % Update navigation
            if obj.Time > obj.AbortTime
                fprintf('    No targets found, returning to base\n')
                obj.MissionFailed = 1;
                obj.FailureType = "Maximum mission time exceeded.";
                State = 'Return to base';
                obj.VelLimit = obj.VelLimitSaved;
            end
        end
        
    case 'Identify'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Identifying target\n',obj.Time)
            obj.IDPosition = obj.States(1:3);
        end
        
        % Update navigation
        Commands = [obj.IDPosition(1:2)' -1 obj.Commands(6)]';
        
        % Update camera
        [Coordinates,DropSite] = obj.CameraModel(obj.States,obj.Time);
        
        % Update object tracking
        [~,TargetFound,~] = obj.ObjectTracking(Coordinates,DropSite);
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % Exit conditions
        if norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-1
            State = 'Hover above target';
%            if obj.WP > 1
%                obj.WP = obj.WP - 1;
%            end
        end
        
    case 'Hover above target'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Moving above target\n',obj.Time)
        end
        
        % Update camera
        [Coordinates,DropSite] = obj.CameraModel(obj.States,obj.Time);
        
        % Update object tracking
        [Centroids,~,TargetVis] = obj.ObjectTracking(Coordinates,DropSite);
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        if TargetVis
            
            % Update navigation
            [Vxy,Centroid] = obj.VisionController(Centroids,obj.StatesEst);
            Commands = [Vxy' obj.Commands(3) obj.Commands(6)]';
            
            % Update controller
            [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
                obj.Controller(obj.StatesEst,Commands,obj.Time,1);
            
        end
        
        % Exit conditions
        if ~TargetVis
            fprintf('    Target lost\n')
            State = 'Search';
        elseif norm(Centroid) < 10 && norm(obj.States(7:9)) < 1e-2
            fprintf('    Above target\n')
            State = 'Descend to grab';
        elseif obj.Time - obj.ModeEntryTime > 10
            fprintf('    Taking too long\n')
            State = 'Search';
            obj.Cint = 0;
        end
        
    case 'Descend to grab'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Descending to grab\n',obj.Time)
        end
        
        % Update camera
        [Coordinates,DropSite] = obj.CameraModel(obj.States,obj.Time);
        
        % Update object tracking
        [Centroids,~] = obj.ObjectTracking(Coordinates,DropSite);
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update navigation
        [Vxy,~] = obj.VisionController(Centroids,obj.StatesEst);
        zd = -(obj.ArmLength+0.09);
        Commands = [Vxy(1:2)' zd  obj.Commands(6)]';
        %                     Commands = [0 0 zd obj.Commands(6)]';
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,1);
        
        % Update geometry
        GPos = obj.GrabberGeometry(obj.States);
        
        % Check grabber position agains targets
        Distance = ones(size(obj.TargetPositions,1),1);
        for i = 1:size(obj.TargetPositions,1)
            Distance(i) = norm(GPos - obj.TargetPositions(i,:)');
        end
        
        % Exit conditions
        %                     if norm(obj.States(7:9)) < 1e-2 && norm(zd-obj.States(3)) < 1e-2
        if any(Distance < 1e-2)
            fprintf('    Target within reach\n')
            State = 'Grab';
            obj.Cint = 0;
        elseif obj.Time - obj.ModeEntryTime > 10
            fprintf('    Taking too long\n')
            State = 'Search';
            obj.Cint = 0;
        end
        
    case 'Grab'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            obj.Camera.Vertices = [];
            fprintf('Time %4.2f s: Grabbing target\n',obj.Time)
        end
        
        % Update navigation
        zd = -(obj.ArmLength+0.1);
        Commands = [0 0 zd obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,1);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % Check grabber position agains targets
        obj.GrabbedTargets = ones(size(obj.TargetPositions,1),1);
        for i = 1:size(obj.TargetPositions,1)
            obj.GrabbedTargets(i) = norm(obj.GrabPosition - obj.TargetPositions(i,:)') < 5e-2;
        end
        
        % Activate grabber
        obj.GrabberActive = 1;
        obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;
        
        % Exit conditions
        if obj.GrabberActive
            obj.GrabberWarning = 0;
            fprintf('    Grabbed successfully\n')
            State = 'Ascend';
        elseif obj.Time - obj.ModeEntryTime > 5
            fprintf('    Taking too long\n')
            State = 'Search';
        end
        
    case 'Ascend'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Ascending to transport\n',obj.Time)
        end
        
        % Update navigation
        Commands = [0 0 -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,1);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % Update grabber
        obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;
        
        % Exit conditions
        if norm(obj.Commands(3)-obj.States(3)) < 1e-2
            fprintf('    At transport height\n')
            State = 'Transport';
        end
        
    case 'Transport'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Transporting target\n',obj.Time)
            
            if strcmp(obj.SearchType,"Pattern")
                x = round(obj.States(1)/obj.CellSize);
                y = round(obj.States(2)/obj.CellSize);
                
                [x, y] = adjustCoords(obj, x, y);
                
                
                obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate*(abs(x) + abs(y));
                fprintf('Transporting decreased battery by %d ; Coordinates = (%d,%d); New battery value: %d \n', 2*(abs(x) + abs(y)), x, y, obj.BatteryLevel)
            end
        end
        
        % Update navigation
        Commands = [obj.DropLocation(1:2)' -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % Update grabber
        obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;
        
%         if obj.Time - obj.ModeEntryTime > 1
%             obj.GrabberActive = 0;
%         end
        
        % Exit conditions
        if norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-2
            fprintf('    Above drop zone\n')
            State = 'Descend to drop';
        end
        
    case 'Descend to drop'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Descending to deposit\n',obj.Time)
        end
        
        % Update navigation
        zd = -(obj.ArmLength+0.1);
        Commands = [obj.DropLocation(1:2)' zd obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % Update grabber
        obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;
        
        % Exit conditions
        if ~obj.GrabberActive
            fprintf('    Target dropped early above drop zone\n')
            State = 'Search';
        elseif norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-2
            fprintf('    Releasing grabber\n')
            State = 'Drop';
        end
        
    case 'Drop'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Dropping target\n',obj.Time)
        end
        
        % Update navigation
        zd = -(obj.ArmLength+0.1);
        Commands = [obj.DropLocation(1:2)' zd obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % Deactivate grabber
        obj.GrabberActive = 0;
        
        % Update target count
        obj.TargetCount = obj.TargetCount + 1;
        
        % Exit conditions
        if obj.TargetCount >= obj.NumTargets
            fprintf('    All targets deposited\n')
%             State = 'Return to base';
            State = 'Land';
            obj.MissionComplete = 1;
        else
            fprintf('    %d targets remaining\n', obj.NumTargets - obj.TargetCount)
            obj.WP = nearestBaseWP(obj);
            obj.Decisions.Mode = 0;
            % Refuel if at base and in Pattern Search
            % Note: SmartSearch handles refueling within module.
            if strcmp(obj.SearchType, "Pattern")
                obj.BatteryUsage = obj.BatteryUsage + (obj.MaxBattery - obj.BatteryLevel);
                obj.BatteryLevel = obj.MaxBattery;
            end
            State = 'Return to search';
        end
        
    case 'Return to search'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Returning to search\n',obj.Time)
        end
        
        % Update navigation
        Commands = [0 0 -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,1);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % Update grabber
        obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;
        
        % Exit conditions
        if norm(obj.Commands(3)-obj.States(3)) < 1e-2
            fprintf('    At search height\n')
            State = 'Search';
        end
        
    case 'Reacquire target'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            obj.Commands(1:3) = obj.States(1:3);
            fprintf('Time %4.2f s: Reacquiring target\n',obj.Time)
        end
        
        % Update camera
        [Coordinates,DropSite] = obj.CameraModel(obj.States,obj.Time);
        
        % Update object tracking
        [Centroids,~] = obj.ObjectTracking(Coordinates,DropSite);
        
        % Update navigation
        Commands = [obj.Commands(1:3)' obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % State transition conditions
        if norm(obj.Commands(1:3)-obj.States(1:3)) < 1e-1 && norm(obj.States(7:9)) < 1e-1
            if ~isempty(Centroids)
                fprintf('    Target reaquired, moving above\n')
                State = 'Hover above target';
            else
                fprintf('    Target lost, returning to search\n')
                State = 'Search';
            end
        end
        
    case 'Return to base'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            obj.GrabberActive = 0;
            fprintf('Time %4.2f s: Returning to base\n',obj.Time)
            
            if strcmp(obj.SearchType,"Pattern")
                x = round(obj.States(1)/obj.CellSize);
                y = round(obj.States(2)/obj.CellSize);
                
                [x, y] = adjustCoords(obj, x, y);
                
                obj.BatteryLevel = obj.BatteryLevel - obj.BatteryLossRate*(abs(x) + abs(y));
                fprintf('Returning to base decreased battery by %d ; Coordinates = (%d,%d); New battery value: %d \n', 2*(x + y), x, y, obj.BatteryLevel)
            end
        end

        
        % Update navigation
        Commands = obj.Waypoints(nearestBaseWP(obj),:)';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % State transition conditions
        if norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-2
            fprintf('    Above base\n')
            State = 'Land';
            obj.WP = nearestBaseWP(obj);
        end
        
    case 'Land'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Landing\n',obj.Time)
        end
        
        % Update navigation
        wp = obj.Waypoints(nearestBaseWP(obj),:);
        Commands = [wp(1:2) -obj.EffRadius obj.Commands(4)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % State transition conditions
        if norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-2 ...
                && norm(obj.States(7:9)) < 1e-2
            fprintf('    Landed\n')
            
            % Refuel if at base and in Pattern Search
            % Note: SmartSearch handles refueling within module.
            if strcmp(obj.SearchType, "Pattern")
                obj.BatteryWarning = 0;
                obj.BatteryUsage = obj.BatteryUsage + (obj.MaxBattery - obj.BatteryLevel);
                obj.BatteryLevel = obj.MaxBattery;
            end
            
            if obj.SystemsOkay
                State = 'Idle';
            else
                obj.MissionFailed = 1;
                obj.FailureType = "System failed to initialize.";
                State = 'Idle';
            end
        end
        
    case 'Emergency land'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Emergency landing\n',obj.Time)
        end
        
        % Update navigation
        Commands = [0 0 -obj.EffRadius 0]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.EmergencyController(obj.StatesEst,Commands,obj.Time,obj.ActuatorFaults);
        
        % Update geometry
        obj.GrabPosition = obj.GrabberGeometry(obj.States);
        
        % State transition conditions
        if obj.States(3) > -1.2*obj.EffRadius
            fprintf('    Crash landed\n')
            State = 'Idle';
            obj.BatteryUsage = obj.BatteryUsage + (obj.MaxBattery - obj.BatteryLevel);
            obj.MissionFailed = 1;
            obj.FailureType = "Actuator fault caused crash land.";
        end
        
end

% Universal updates
obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;
