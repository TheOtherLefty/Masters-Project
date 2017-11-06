function State = StateMachine(obj,State)

% Check for mode changes
if ~strcmp(State,obj.PrevMode)
    obj.ModeEntryTime = obj.Time;
    obj.ModeTransition = 'Entry';
end
obj.PrevMode = State;

switch State
    
    case 'Idle'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            obj.BatteryRate = obj.ChargeRate;
            fprintf('Time %4.2f s: Agent idle\n',obj.Time)
            if obj.MissionComplete
                fprintf('    MISSION COMPLETE\n')
            elseif obj.MissionFailed
                fprintf('    MISSION FAILED\n')
            end
        end
        
        % Update navigation
        %                     obj.Commands = NaN*ones(12,1);
        
        % Update controller
        obj.Inputs = [0 0 0 0]';
        obj.epos_int = [0 0 0]';
        
        % State transition conditions
        if ~obj.MissionComplete && ~obj.MissionFailed && obj.States(15) >= obj.MaxBattery
            State = 'Take-off';
            obj.BatteryWarning = 0;
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
        
        % Update navigation
        Commands = [obj.Home(1:2)' -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        % State transition conditions
        if norm(obj.Commands(1:3) - obj.States(1:3)) < 1e-1
            obj.MissionComplete = 0;
            State = 'Search';
        end
        
    case 'Initialise'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Initialising\n',obj.Time)
        end
        
        % Update navigation
        Commands = [obj.Home(1:2)' -1 obj.Commands(6)]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
        obj.SystemsOkay = rand < (1 - obj.P_SystemsFault);
        
        % Force system fault
        %                     if obj.Time < 3
        %                         obj.SystemsOkay = 0;
        %                     end
        
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
        end
        
        % Update navigation
        if isempty(obj.Decisions)
            [Commands,ExitFlag] = obj.SearchPattern(obj.States,obj.Time-obj.ModeEntryTime);
        else
            [Commands,ExitFlag] = obj.SmartSearch(obj.States,obj.Time-obj.ModeEntryTime);
        end
        
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
        if TargetFound
            fprintf('    Target found, need to identify\n')
            State = 'Identify';
            obj.VelLimit = obj.VelLimitSaved;
        elseif ExitFlag || obj.Time > obj.AbortTime
            fprintf('    No targets found, returning to base\n')
            obj.MissionFailed = 1;
            State = 'Return to base';
            obj.VelLimit = obj.VelLimitSaved;
        end
        
    case 'Identify'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Identifying target\n',obj.Time)
            obj.IDPosition = obj.States(1:3);
        end
        
        % Update navigation
        Commands = [obj.IDPosition(1:2)' -2 obj.Commands(6)]';
        
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
            if obj.WP > 1
                obj.WP = obj.WP - 1;
            end
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
            fprintf('    Targets remaining\n')
            State = 'Return to search';
        end
        
    case 'Return to search'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Returning to search\n',obj.Time)
        end
        
        % Update navigation
        Commands = [0 0 -2 obj.Commands(6)]';
        
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
        end
        
        % Update navigation
        Commands = [obj.Home(1:2)' -1 0]';
        
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
        end
        
    case 'Land'
        
        % Entry conditions
        if strcmp(obj.ModeTransition,'Entry')
            obj.ModeTransition = 'Active';
            fprintf('Time %4.2f s: Landing\n',obj.Time)
        end
        
        % Update navigation
        Commands = [obj.Home(1:2)' -obj.EffRadius obj.Commands(4)]';
        
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
            State = 'Idle';
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
            obj.MissionFailed = 1;
        end
        
end

% Universal updates
obj.GrabbedTargets = obj.GrabbedTargets*obj.GrabberActive;

% Fault times
