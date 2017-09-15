classdef cQuadrotor < cAgent
    %HEXAROTOR Summary of this class goes here
    %   Detailed explanation goes here

    properties

        % Quadrotor properties
        mQ = 1.51;      % Mass (kg)
        g = 9.81;   % Acceleration due to gravity (m/s2)
        Ix = 0.03;   % Inertia about x-axis (kg m2)
        Iy = 0.03;   % Inertia about y-axis (kg m2)
        Iz = 0.04;   % Inertia about z-axis (kg m2)
        L = 0.2;    % Rotor distance from CG (m)
        Kt = 120;    % Rotor thrust gain (N)
        Kq = 2;     % Rotor torque gain (N m)
        Gtau = 0.005; % Gimbal dynamics time constant
        
        % Object properties
        mO = 0.1;

        % Floor properties
        FloorNatFreq = 50;
        FloorDamp = 0.5;

        % Power properties
        MaxBattery = 11;
        ChargeRate = 0.025;
        LossRate = -0.005;
        BatteryRate;
        WarningLevel = 10.55;

        % Controller properties
        Kx;
        Ky;
        Kz;
        Kphi;
        Ktheta;
        Kpsi;
        AttLimit = 20*pi/180;
        AccLimit = 2;
        VelLimit = 5;
        VelLimitSaved = 5;
        dt_Controller = 0.01;
        VelFilter = 50;
        VelInt = [0 0 0]';
        Cint = 0;
        ActuatorLimits = 1;

        % Sensor properties
        dt_Sensor = 0.01;
        CameraPosition = [0 0 0.1]';
        CameraAngle = -90*pi/180;
        Camera
        n_Acc = 1e-2;
        n_Gyro = 1e-3;
        n_OT = 1e-4;

        % Mission properties
        SystemsOkay
        MissionComplete
        MissionFailed = 0;
        MissionEnd = 0;
        AbortTime = 300;
        ModeEntryTime = 0;
        Waypoints
        IDPosition;
        WP = 1;
        MaxWP;
        CurrentColours;
        NC = 1;
        SearchColours;
        Cvar;
        Ccheck;
        DropLocation;
        DropRadius;
        Home;
        NumTargets = 0;
        TargetCount = 0;

        % Grabber properties
        GrabberActive = 0;
        ArmLength = 0.2;
        TargetPositions;
        GrabPosition;
        GrabbedTargets;

        % Warnings
        ActuatorFaultWarning = 0;
        GrabberWarning = 0;
        SystemsWarning = 0;
        BatteryWarning = 0;

        % Fault properties
        % (Faults % probability occ. in 1 minute)
        P_GrabberFailure = 0.05;
        P_ActuatorFailure = 0.01;
        P_SystemsFault = 0.05;
        ActuatorFaults = [0 0 0 0]';

    end

    methods

        % CLASS CONSTRUCTOR -----------------------------------------------
        function obj = cQuadrotor(ID,Environment,varargin)

            % ENVIRONMENT PROPERTIES
            obj.RoomLimits = Environment.RoomLimits;

            % AGENT PROPERTIES --------------------------------------------

            obj.ID = ID;

            % Initialise time
            obj.Time = 0;

            % Initialise time-step
            i = find(strcmp(varargin,'Time-step'));
            if isempty(i)
                obj.TimeStep = 0.01;
            else
                obj.TimeStep = varargin{i+1};
            end

            % Initialise position
            obj.EffRadius = 0.2;
            i = find(strcmp(varargin,'Pose'));
            if isempty(i)
                Pose = obj.InitPosition;
            else
                Pose = varargin{i+1};
            end

            % Set home location
            obj.Home = Pose(1:3);
            obj.VelInt = Pose(1:3);

            % Intialise states
            obj.States = [Pose
                          zeros(6,1)
                          zeros(2,1)
                          obj.MaxBattery
                          Pose(1:3) + [0 0 obj.ArmLength]';
                          zeros(3,1)];

            % Initialise commands
            obj.Commands = NaN*ones(12,1);
            obj.Commands(6) = Pose(6);

            % Initialise controller
            obj.InitialiseControllers;
            obj.Inputs = [0 0 0 0]';
            obj.PseudoInputs = [0 0 0 0]';

            % Initialise battery
            obj.BatteryRate = obj.ChargeRate;

            % Initialise state transitions
            obj.StateTrans = obj.Dynamics(obj.States,obj.Inputs,obj.Time);

            % Initialise sensors
            obj.Outputs...
                = obj.Sensors(obj.Outputs,obj.States,obj.StateTrans,obj.Time);

            % Set initial state
            i = find(strcmp(varargin,'Mode'));
            if isempty(i)
                obj.Mode = 'Idle';
            else
                obj.Mode = varargin{i+1};
            end
            obj.PrevMode = obj.Mode;

            % Set initial status
            obj.Status = 'Fine';

            % Set initial height to effective height if at Idle
            if strcmp(obj.Mode,'Idle')
                obj.States(3) = -obj.EffRadius;
            end

            % Initialise geometry
            obj.Geometry = obj.InitialiseGeometry;
            obj.Geometry = obj.UpdateGeometry(obj.Geometry,obj.States);

            % Set integration algorithm
            i = find(strcmp(varargin,'Solver'));
            if isempty(i)
                obj.Solver = 'RK4';
            else
                obj.Solver = varargin{i+1};
            end

            % Initialise camera
            obj.Camera.FOV = 45*pi/180;
            obj.Camera.Res = [800 600];
            obj.Camera.Bnd = obj.Camera.Res*2/3;
            obj.Camera.BndRadius = 100;
            obj.Camera.Vertices = [];
            obj.Camera.SceneVertices = [];
            obj.Camera.DropSite = [];
            obj.Camera.FocalLength = obj.Camera.Res(1)/(2*tan(obj.Camera.FOV));
            obj.Camera.Centroid = NaN*[1 1 1];

            % MISSION PROPERTIES ------------------------------------------

            obj.SystemsOkay = 1;
            obj.MissionComplete = 0;

            % Search colours
            obj.SearchColours = [0.7 0 0
                                 0 0.7 0
                                 0 0 0.7];
            obj.CurrentColours = obj.SearchColours;
            obj.Cvar = 0.3;

        end

        % UPDATE AGENT
        function BB = UpdateAgent(obj,BB,ind)

            % Set agent index
            if isempty(obj.Index)
                obj.Index = ind;
            end

            % Load scene geometry
            obj.Camera.SceneVertices = BB.Geometry.Vertices;
            obj.Camera.Faces = BB.Geometry.Faces;
            obj.Camera.Colours = BB.Geometry.Colours;
            obj.Camera.SceneDropSite = BB.Geometry.DropSite;
            obj.DropLocation = BB.Geometry.DropLocation;
            obj.DropRadius = BB.Geometry.DropRadius;

            % Get target positions
            for a = 1:length(BB.Agents.Target)
                obj.TargetPositions(a,:) = BB.Agents.Target(a).States(1:3)' - [0 0 0.05];
            end

            % Update finite states (override in testing mode)
            if rem(obj.Time,obj.dt_Controller) < 1e-6
                if ~strcmp(obj.Mode,'Testing')

                    obj.Mode = obj.StateMachine(obj.Mode);

                else

                    Commands = [0 0 -0.5 pi/2]';

                    % State reconstruction
                    obj.StatesEst = obj.StateReconstruction(obj.Outputs);

                    % Update controller
                    [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
                        obj.Controller(obj.StatesEst,Commands,obj.Time,0);

                end
            end

            % Update states
            [obj.States,obj.StateTrans,obj.Time] = ...
                obj.Integrate(obj.States,obj.Inputs,obj.Time);

            % Update sensors
            obj.Outputs = ...
                obj.Sensors(obj.Outputs,obj.States,obj.StateTrans,obj.Time);

            % Update geometry
            obj.Geometry = obj.UpdateGeometry(obj.Geometry,obj.States);

            % Update faults
            obj.Faults;
            obj.Mode = obj.FaultDetection(obj.States,obj.Mode);

            % Update status monitor
            obj.Status = obj.StatusMonitor;

            % Update blackboard
            Props = {'Time','States','StateTrans','Inputs','PseudoInputs',...
                'Outputs','Commands','Mode','Status'};
            for p = 1:length(Props)
                BB.Agents.Quad(ind).(Props{p}) = obj.(Props{p});
            end

            % Update geometry
            ID = obj.Geometry.ID;
            BB.Geometry.Vertices(ID:ID+size(obj.Geometry.Vertices,1)-1,:)...
                = obj.Geometry.Vertices;
            BB.Agents.Quad(ind).Camera.Vertices = obj.Camera.Vertices;
            BB.Agents.Quad(ind).Camera.Centroid = obj.Camera.Centroid;
            BB.Agents.Quad(ind).Camera.DropSite = obj.Camera.DropSite;

            % Update grabber properties
            BB.Agents.Quad(ind).GrabPosition = obj.GrabPosition;
            BB.Agents.Quad(ind).GrabbedTargets = obj.GrabbedTargets;

        end

        %------------------------------------------------------------------
        % AGENT DYNAMIC MODEL
        %------------------------------------------------------------------

        % VEHICLE DYNAMICS
        function Xdot = Dynamics(obj,X,U,t)

            % Continuous states
            % X = [x y z phi theta psi xdot ydot zdot p q r]'

            % Rotation matrices
            Rphi = [1  0         0
                    0  cos(X(4)) sin(X(4))
                    0 -sin(X(4)) cos(X(4))];
            Rtheta = [cos(X(5)) 0 -sin(X(5))
                      0         1  0
                      sin(X(5)) 0  cos(X(5))];
            Rpsi = [ cos(X(6)) sin(X(6)) 0
                    -sin(X(6)) cos(X(6)) 0
                     0         0         1];
            Reb = Rphi*Rtheta*Rpsi;
            Rbe = Reb';

            J = [1  0         -sin(X(5))
                 0  cos(X(4))  sin(X(4))*cos(X(5))
                 0 -sin(X(4))  cos(X(4))*cos(X(5))];
            
            % Update mass and inertias if object attached
            if obj.GrabberActive
                m = obj.mQ + obj.mO;
            else
                m = obj.mQ;
            end

            % Inertias
            I = diag([obj.Ix obj.Iy obj.Iz]);

            % ROTOR MODEL

            % Fault model
            U = ~obj.ActuatorFaults.*U;

            % Limit inputs
            if obj.ActuatorLimits
                U(U<0) = 0;
                U(U>0.05) = 0.05;
            end

            % Individual thrust and torque
            Ti = obj.Kt*U;
            Qi = obj.Kq*U.*[-1 -1 1 1]';

            % Net thrust and torque
            T = sum(Ti);
            Q = sum(Qi);

            % Rotor positions around CG
            Ri = [-obj.L obj.L 0 0
                  0 0 -obj.L obj.L
                  0 0 0 0];

            % GROUND MODEL

            % Floor spring and damping
            kf = m*obj.FloorNatFreq^2;
            cf = m*2*obj.FloorDamp*obj.FloorNatFreq;

            if X(3) > -obj.EffRadius
                G = Reb*[-cf*X(7:8)
                          kf*(-obj.EffRadius-X(3)) - cf*X(9)];
                Gm = -0.01*[kf*X(4:5) + cf*X(10:11)
                           cf*X(12)];
            else
                G = [0 0 0]';
                Gm = [0 0 0]';
            end

            % FORCES AND MOMENTS

            % Forces
            F = [0 0 -T]' + G;

            % Moments
            M = [0 0 0]' + Gm;
            for i = 1:length(U)
                M = M + cross(Ri(:,i),[0 0 -Ti(i)]') + [0 0 Qi(i)]';
            end

            % BATTERY MODEL
            if X(15) >= obj.MaxBattery && obj.BatteryRate > 0
                Vrate = 0;
            elseif X(15) <= 0 && obj.BatteryRate < 0
                Vrate = 0;
            else
                Vrate = obj.BatteryRate;
            end

            % Grabber dynamics
            rB = [0 0 obj.ArmLength]';

            % STATE DERIVATIVES
            Xdot = [X(7:9)
                    J\X(10:12)
                    Rbe*F/m + [0 0 obj.g]';
                    I\M
                    (-X(4:5)-X(13:14))/obj.Gtau
                    Vrate
                    X(19:21)
                    Rbe*F/m + [0 0 obj.g]' + Rbe*(cross(X(10:12),cross(X(10:12),rB)) + cross(I\M,rB))];

            % Checks
%             if obj.Time > 140 && strcmp(obj.Mode,'Search')
%                 fprintf('Time = %.2f s\n',t)
%                 fprintf('Commands = ['),fprintf('%.3f ',obj.Commands([1:3, 6])),fprintf(']\n')
%                 fprintf('Inputs = [ '),fprintf('%.3f ',U),fprintf(']\n')
%                 fprintf('Forces = [ '),fprintf('%.3f ',F),fprintf(']\n')
%                 fprintf('Floor = [ '),fprintf('%.3f ',G),fprintf(']\n')
%                 fprintf('Moments = [ '),fprintf('%.3f ',M),fprintf(']\n')
%                 fprintf('Floor = [ '),fprintf('%.3f ',Gm),fprintf(']\n')
%                 fprintf('Pos = [ '),fprintf('%.3f ',X(1:3)),fprintf(']\n')
%                 fprintf('Vel = [ '),fprintf('%.3f ',X(4:6)),fprintf(']\n')
%                 fprintf('Acc = [ '),fprintf('%.3f ',Xdot(4:6)),fprintf(']\n\n')
%                 pause
%             end

        end

        %------------------------------------------------------------------
        % CONTROL AND NAVIGATION
        %------------------------------------------------------------------

        % FINITE STATE MACHINE
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
                        State = 'Initialise';
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
                    [Commands,ExitFlag] = obj.SearchPattern(obj.States,obj.Time-obj.ModeEntryTime);

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

%                     if obj.Time - obj.ModeEntryTime > 1
%                         obj.GrabberActive = 0;
%                     end

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
                        State = 'Return to base';
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
            

        end

        % CONTROL ---------------------------------------------------------

        % CONTROLLER
        function [U,Up,Yd] = Controller(obj,X,Yd,t,Vswitch)

            % Reconstruct states
            Pos = X(1:3);
            Att = X(4:6);
            Vel = X(7:9);
            AngVel = X(10:12);

            % Switch between position and velocity control
            AttDes = [0 0 wrapToPi(Yd(4))]';
            AngVelDes = [0 0 0]';

            % Position controller gains
            Kp = [obj.Kx(1) obj.Ky(1) obj.Kz(1)]';
            Kd = [obj.Kx(2) obj.Ky(2) obj.Kz(2)]';

            if Vswitch
                PosDes = [NaN NaN Yd(3)]';
                VelDes = [Yd(1:2)
                          Kp(3)*(PosDes(3) - Pos(3))];
            else
                PosDes = Yd(1:3);
                VelDes = Kp.*(PosDes - Pos);
            end

            % Limit velocity
            VelDes = VelDes.*(abs(VelDes)<=obj.VelLimit) + obj.VelLimit*(VelDes>obj.VelLimit)...
                - obj.VelLimit*(VelDes<-obj.VelLimit);

            % Velocity controller
            Acc = Kd.*(VelDes - Vel);

            % Limit acceleration
            Acc = Acc.*(abs(Acc)<=obj.AccLimit) + obj.AccLimit*(Acc>obj.AccLimit)...
                - obj.AccLimit*(Acc<-obj.AccLimit);

            uz = obj.mQ*(obj.g - Acc(3))/(obj.Kt*cos(Att(1))*cos(Att(2)));
%             AttDes(1) = -(vx*sin(Att(3)) - vy*cos(Att(3)))/obj.g;
%             AttDes(2) = -(vx*cos(Att(3)) + vy*sin(Att(3)))/obj.g;

            AttDes(1) = asin(obj.mQ*(Acc(2)*cos(Att(3)) - Acc(1)*sin(Att(3)))/(obj.Kt*uz));
            AttDes(2) = -asin(obj.mQ*(Acc(1)*cos(Att(3)) + Acc(2)*sin(Att(3)))/(obj.Kt*uz*cos(Att(1))));

            for i = 1:2
                if abs(AttDes(i)) > obj.AttLimit
                    AttDes(i) = sign(AttDes(i))*obj.AttLimit;
                end
            end

            % Attitude control
            vphi = obj.Kphi*[AttDes(1) - Att(1);
                             AngVelDes(1) - AngVel(1)];
            vtheta = obj.Ktheta*[AttDes(2) - Att(2);
                             AngVelDes(2) - AngVel(2)];
            vpsi = obj.Kpsi*[AttDes(3) - Att(3);
                             AngVelDes(3) - AngVel(3)];

            uphi = obj.Ix*vphi/(obj.Kt*obj.L);
            utheta = obj.Iy*vtheta/(obj.Kt*obj.L);
            upsi = obj.Iz*vpsi/obj.Kq;

            Up = [uz uphi utheta upsi]';

            % Get true rotor inputs from pseudo-controls
            C = [ 1  1 1  1
                  0  0 1 -1
                 -1  1 0  0
                 -1 -1 1  1];
            Cinv = C'/(C*C');

            U = Cinv*Up;

            % Limit inputs
            if obj.ActuatorLimits
                U(U<0) = 0;
                U(U>0.05) = 0.05;
            end

            % Output commands
            Yd = [PosDes; AttDes; VelDes; AngVelDes];

        end

        % INITIALISE CONTROLLERS
        function InitialiseControllers(obj)

            % CLOSED-LOOP RESPONSE CHARACTERISTICS

            % Height
            tsz = 2;
            zetaz = 1;
            wnz = 3.9/(tsz*zetaz);

            obj.Kz(2) = 2*zetaz*wnz;
            obj.Kz(1) = wnz^2/obj.Kz(2);

            % Position
            tsp = 2;
            zetap = 1;
            wnp = 3.9/(tsp*zetap);

            obj.Kx(2) = 2*zetap*wnp;
            obj.Kx(1) = wnp^2/obj.Kx(2);
            obj.Ky = obj.Kx;

            % Roll and pitch
            zetaa = 1;
            wna = 10*wnp;
            tsa = 3.9/(zetaa*wna);

            obj.Kphi(1) = wna^2;
            obj.Kphi(2) = 2*zetaa*wna;
            obj.Ktheta = obj.Kphi;

            % Yaw
            tsy = 4;
            zetay = 1;
            wny = 3.9/(zetay*tsy);

            obj.Kpsi(1) = wny^2;
            obj.Kpsi(2) = 2*zetay*wny;

        end

        % RECONSTRUCT STATES FROM SENSOR MEASUREMENTS
        function X = StateReconstruction(obj,Y)

            % Position
            X(1:3,1) = Y(1:3);

            % Attitude
            X(4:6,1) = Y(4:6);

            % Velocity
            X(7:9,1) = obj.VelFilter*(Y(1:3) - obj.VelInt);
            obj.VelInt = obj.VelInt + obj.dt_Controller*X(7:9);

            % Angular velocity
            X(10:12,1) = Y(10:12);

        end

        % VISUAL CONTROLLER
        function [Vxy,C] = VisionController(obj,Coords,X)

            % Align camera axes with body
            C = [Coords(1,2) Coords(1,1)]';

            % Scale error
            Cs = C*abs((X(3)-obj.CameraPosition(3)))/obj.Camera.FocalLength;

            % Gains
            Kp = 0.35*obj.Kx(1);
            Ki = 0.0001;

            % Integral error
            obj.Cint = obj.Cint + obj.dt_Controller*Cs;


            % Center coords in frame of reference
            Vxy = Kp*Cs + Ki*obj.Cint;

            % Fix frame of reference
            Rpsi = [cos(X(6)) -sin(X(6))
                    sin(X(6))  cos(X(6))];
            Vxy = Rpsi*Vxy;

        end

        % EMERGENCY CONTROLLER
        function [U,Up,Yd] = EmergencyController(obj,X,Yd,t,Faults)

            % Reconstruct states
            Pos = X(1:3);
            Att = X(4:6);
            Vel = X(7:9);
            AngVel = X(10:12);

            % Switch between position and velocity control
            AttDes = [0 0 wrapToPi(Yd(4))]';
            AngVelDes = [0 0 0]';

            % Position controller gains
            Kp = [obj.Kx(1) obj.Ky(1) obj.Kz(1)]';
            Kd = [obj.Kx(2) obj.Ky(2) obj.Kz(2)]';

            PosDes = [NaN NaN Yd(3)]';
            VelDes = [Yd(1:2)
                      Kp(3)*(PosDes(3) - Pos(3))];

            % Limit velocity
            VelDes = VelDes.*(abs(VelDes)<=obj.VelLimit) + obj.VelLimit*(VelDes>obj.VelLimit)...
                - obj.VelLimit*(VelDes<-obj.VelLimit);

            % Velocity controller
            Acc = Kd.*(VelDes - Vel);

            % Limit acceleration
            Acc = Acc.*(abs(Acc)<=obj.AccLimit) + obj.AccLimit*(Acc>obj.AccLimit)...
                - obj.AccLimit*(Acc<-obj.AccLimit);

            uz = obj.mQ*(obj.g - Acc(3))/(obj.Kt*cos(Att(1))*cos(Att(2)));
%             uz = 2*obj.m*(obj.g - Acc(3))/(obj.Kt*cos(Att(1))*cos(Att(2)));

            Up = [uz 0 0 0]';

            % Get true rotor inputs from pseudo-controls
            U = uz*0.5*[1 1 1 1]';
            if any(find(Faults) == [1,2])
                U(1:2) = [0 0];
            elseif any(find(Faults) == [3,4])
                U(3:4) = [0 0];
            else
                error('This shouldn''t happen')
            end

            % Limit inputs
            U(U<0) = 0;
            U(U>0.05) = 0.05;

            % Output commands
            Yd = [PosDes; AttDes; VelDes; AngVelDes];

        end

        % NAVIGATION ------------------------------------------------------

        % SEARCH MODE TRAJECTORY
        function [Yd,Exitflag] = SearchPattern(obj,Y,t)

            % Track waypoints
            if norm(Y([1:3,6])-obj.Waypoints(obj.WP,1:4)') < 1e-1
                obj.WP = obj.WP+1;
            end

            if obj.WP > obj.MaxWP
                obj.WP = obj.MaxWP;
                Exitflag = 1;
            else
                Exitflag = 0;
            end

            Yd = obj.Waypoints(obj.WP,:)';

        end

        % INITIALISE WAYPOINTS
        function InitWaypoints(obj)

            % Define waypoints
            dR = 0.5;

            Waypoints = [obj.RoomLimits(1,1)+dR obj.RoomLimits(2,1)+dR -2 pi/2];
            i = 1;

            while (max(Waypoints(:,1)) < obj.RoomLimits(1,2)-dR) || (Waypoints(i,2) ~= obj.RoomLimits(2,2)-dR)

                i = i + 1;

                Waypoints(i,:) = Waypoints(i-1,:);

                if rem(i,2) > 1e-3
                    Waypoints(i,1) = Waypoints(i-1,1) + dR;
                    if rem(i+1,4) > 1e-3
                        Waypoints(i,4) = pi/2;
                    else
                        Waypoints(i,4) = -pi/2;
                    end
                else
                    if rem(i,4) > 1e-3
                        Waypoints(i,2) = obj.RoomLimits(2,2)-dR;
                    else
                        Waypoints(i,2) = obj.RoomLimits(2,1)+dR;
                    end
                end

            end

            obj.Waypoints = Waypoints;
            obj.MaxWP = i;

        end

        % IDENTIFY AND TRACK TARGET ---------------------------------------
        function [Cvisd,TargetFound,TargetVis] = ObjectTracking(obj,Vc,Vdsc)

            % Get scene geometry info
            F = obj.Camera.Faces;
            C = obj.Camera.Colours;
            res = obj.Camera.Res;

            Cup = obj.CurrentColours + obj.Cvar;
            Clw = obj.CurrentColours - obj.Cvar;
            numC = size(obj.CurrentColours,1);

            % Preallocate arrays
            Fid = cell(1,numC);
            Fvis = cell(1,numC);
            Cvis = cell(1,numC);

            % Find indices of faces within search range
            for c = 1:numC
                Fidc = zeros(size(C));
                for j = 1:3
                    Fidc(:,j) = and(C(:,j) <= Cup(c,j),C(:,j) >= Clw(c,j));
                end
                Fid{c} = all(Fidc,2);

                % Eliminate faces outside of range
                Fvis{c} = F(Fid{c},:);
                Cvis{c} = C(Fid{c},:);
            end

            % Find vertices corresponding to these faces and get centroid
            Vu = cell(1,numC);
            Cntrd = zeros(size(obj.CurrentColours));
            obj.Ccheck = zeros(size(obj.CurrentColours,1),1);
            for c = 1:numC

                Fvisc = reshape(Fvis{c},[],1);
                Fvisc(isnan(Fvisc)) = [];

                % Find vertices
                Vu{c} = Vc(Fvisc,:);

                % Eliminate vertices outside of FOV
                Vout = [];
                for i = 1:size(Vu{c},1)
                    if Vu{c}(i,1) > -res(1)/2 && Vu{c}(i,1) < res(1)/2 ...
                            && Vu{c}(i,2) > -res(2)/2 && Vu{c}(i,2) < res(2)/2 ...
                            && Vu{c}(i,3) > 0
                        Vout = [Vout
                               Vu{c}(i,:)];
                    end
                end
                Vu{c} = Vout;

                if ~isempty(Vu{c})
                    Cntrd(c,:) = sum(Vu{c},1)/size(Vu{c},1);
                    obj.Ccheck(c,1) = 1;
                else
                    Cntrd(c,:) = NaN*[1 1 1];
                    obj.Ccheck(c,1) = 0;
                end

            end

            % Save centroids to camera property
            obj.Camera.Centroid = Cntrd;

            % Remove NaNs from array
            Cvis = Cntrd(~all(isnan(Cntrd),2),:);

            % Remove centroids in drop site
            % Estimate drop radius, do this properly later
            DS = sum(Vdsc,2)/size(Vdsc,2);
            CR = obj.Camera.FocalLength*obj.DropRadius/abs((obj.States(3)-obj.CameraPosition(3)));
            Cvisd = [];
            D = [];
            for c = 1:size(Cvis,1)
                D(c) = norm(Cvis(c,1:2)-DS(1:2)');
            end
            if ~isempty(D)
                Cvisd = Cvis(D>CR,:);
            end

            % If target is within field of view and outside drop zone, it
            % is visible
            if isempty(Cvisd)
                TargetVis = 0;
            else
                TargetVis = 1;
            end

            % Remove centroids outside of boundary
            Cvisb = [];
            B = [];
            for c = 1:size(Cvisd,1)
                B(c) = norm(Cvisd(c,1:2));
            end
            if ~isempty(B)
                Cvisb = Cvisd(B<obj.Camera.BndRadius,:);
            end
%             Cvisb = Cvis(and(abs(Cvis(:,1))<obj.Camera.Bnd(1)/2,...
%                 abs(Cvis(:,2))<obj.Camera.Bnd(2)/2),:);

            % Sort centroids by distance from camera centre
            Cnorm = zeros(size(Cvisd,1),1);
            for c = 1:size(Cvisd,1)
                Cnorm(c,1) = norm(Cvis(c,1:2));
            end
            if ~isempty(Cvisd)
                Cvisd = [Cvisd Cnorm];
                Cvisd = sortrows(Cvisd,4);
                Cvisd = Cvisd(:,1:3);
            end

            % If no centroids are left, target is not found
            if isempty(Cvisb)
                TargetFound = 0;
            else
                TargetFound = 1;
            end

            if obj.Time > 163 && obj.Time < 170 && 0
                fprintf('Time = %.2f s\n',obj.Time)
                fprintf('TargetFound = %.0f\n',TargetFound)
                fprintf('DropSite = [ ')
                fprintf('%.3f ',DS')
                fprintf(']\n')
                fprintf('Cvis = [ ')
                fprintf('%.3f ',Cvis')
                fprintf(']\n')
                fprintf('Cvisd = [ ')
                fprintf('%.3f ',Cvisd')
                fprintf(']\n')
                fprintf('Cvisb = [ ')
                fprintf('%.3f ',Cvisb')
                fprintf(']\n\n')
            end

            % Remove entry from array
%             obj.CurrentColours = obj.CurrentColours(~obj.Ccheck,:);

        end

        %------------------------------------------------------------------
        % SENSORS
        %------------------------------------------------------------------

        % SENSORS
        function Y = Sensors(obj,Y,X,Xdot,t)

            % Continuous states
            % X = [x y z phi theta psi xdot ydot zdot p q r]'

            % Outputs
            % Y = [ax ay az gx gy gz]'

            % Optitrack
            Y(1:6,1) = obj.Optitrack(X,t);

            % IMU
            Y(7:12,1) = obj.IMU(X,Xdot,t);

        end

        % OPTITRACK MOTION CAPTURE
        function Y = Optitrack(obj,X,t)

            Y = X(1:6);

        end

        % INERTIAL MEASUREMENT UNIT
        function Y = IMU(obj,X,Xdot,t)

            % Rotation matrices
            Rphi = [1  0         0
                    0  cos(X(4)) sin(X(4))
                    0 -sin(X(4)) cos(X(4))];
            Rtheta = [cos(X(5)) 0 -sin(X(5))
                      0         1  0
                      sin(X(5)) 0  cos(X(5))];
            Rpsi = [ cos(X(6)) sin(X(6)) 0
                    -sin(X(6)) cos(X(6)) 0
                     0         0         1];
            Reb = Rphi*Rtheta*Rpsi;
%             Rbe = Reb';

            % Accelerometer
            Y(1:3,1) = Reb*(Xdot(7:9) - [0 0 obj.g]');

            % Gyroscope
            Y(4:6,1) = X(10:12);

            % Magnetometer

        end

        % CAMERA SENSOR
        function [Vc,Vdsc] = CameraModel(obj,X,t)

            % Transform geometry to quadrotor frame
            V = obj.Camera.SceneVertices;
            V = obj.TransformtoBodyAxes(V',X(1:3),X(4:6));
            V = V';

            % Transform geometry to camera frame
            V = obj.TransformtoBodyAxes(V',obj.CameraPosition,[0 obj.CameraAngle 0]');

            % Stabilise camera
            V = obj.TransformtoBodyAxes(V,[0 0 0]',[X(13:14)' 0]');
%             V = obj.TransformtoBodyAxes(V,[0 0 0]',[-X(4:5)' 0]');
            V = V';

            % Convert to image space
            f = obj.Camera.FocalLength;
            Vc = [ f*V(:,2)./V(:,1)...
                  -f*V(:,3)./V(:,1)...
                   f*V(:,1)];

            % Save to camera object
            obj.Camera.Vertices = Vc;

            % Transform dropsite geometry to quadrotor frame
            Vds = obj.Camera.SceneDropSite;
            Vds = obj.TransformtoBodyAxes(Vds,X(1:3),X(4:6));

            % Transform dropsite geometry to camera frame
            Vds = obj.TransformtoBodyAxes(Vds,obj.CameraPosition,[0 obj.CameraAngle 0]');

            % Stabilise camera
            Vds = obj.TransformtoBodyAxes(Vds,[0 0 0]',[X(13:14)' 0]');
%             Vds = obj.TransformtoBodyAxes(Vds,[0 0 0]',[-X(4:5)' 0]');

            % Convert to image space
            Vds = Vds';
            Vdsc = [ f*Vds(:,2)./Vds(:,1)...
                    -f*Vds(:,3)./Vds(:,1)...
                     f*Vds(:,1)]';

            % Save to camera object
            obj.Camera.DropSite = Vdsc;

%             pause

        end

        %------------------------------------------------------------------
        % GEOMETRY
        %------------------------------------------------------------------

        % INITIALISE
        function Geo = InitialiseGeometry(obj)

            load('QuadrotorGeoModel.mat')
            Geo = GeoModel;
            Geo.BodyVertices = GeoModel.Vertices;

        end

        % GRABBER GEOMETRY
        function GPos = GrabberGeometry(obj,X)

            GPosBody = [0 0 obj.ArmLength]';
            GPos = obj.TransformtoEarthAxes(GPosBody,X(1:3),X(4:6));

        end

        %------------------------------------------------------------------
        % FAULTS
        %------------------------------------------------------------------

        % FAULTS
        function Faults(obj)

            % Grabber fault, can occur at any time but only makes a
            % difference when transporting objects
            if rand > (1 - obj.P_GrabberFailure)^(obj.TimeStep/60)
                obj.GrabberActive = 0;
            end

            % Actuator fault, can occur at any time
            if ~all(obj.ActuatorFaults) && rand > (1 - obj.P_ActuatorFailure)^(obj.TimeStep/60)
                ind = ceil(4*rand);
                obj.ActuatorFaults(ind) = 1;
            end

        end

        % FAULT DETECTION
        function State = FaultDetection(obj,X,State)

            % Battery monitor
            if X(15) < obj.WarningLevel && ~obj.BatteryWarning
                fprintf('Time %4.2f s: BATTERY WARNING\n',obj.Time)
                obj.Cint = 0;
                State = 'Return to base';
                obj.BatteryWarning = 1;
            end

            % Grabber fault
            if ~obj.GrabberActive && ismember(State,{'Ascend','Transport'})
                fprintf('Time %4.2f s: TARGET DROPPED\n',obj.Time)
                State = 'Reacquire target';
                obj.GrabberWarning = 1;
            end

            % Actuator fault monitor
            if any(obj.ActuatorFaults) && ~obj.ActuatorFaultWarning
                fprintf('Time %4.2f s: FAULTY ACTUATOR\n',obj.Time)
                State = 'Emergency land';
                obj.ActuatorFaultWarning = 1;
            end

        end

        % STATUS MONITOR
        function Status = StatusMonitor(obj)

            if obj.ActuatorFaultWarning
                Status = 'Actuator fault';
            elseif obj.GrabberWarning
                Status = 'Grabber fault';
            elseif obj.BatteryWarning
                Status = 'Low battery';
            elseif obj.SystemsWarning
                Status = 'System fault';
            else
                Status = 'Fine';
            end

        end

    end

    methods(Static)

        % Transform positions and vertices to body axes
        function V = TransformtoBodyAxes(V,Pos,Att)

            % Rotation matrices
            Rphi = [1  0         0
                    0  cos(Att(1)) sin(Att(1))
                    0 -sin(Att(1)) cos(Att(1))];
            Rtheta = [cos(Att(2)) 0 -sin(Att(2))
                      0         1  0
                      sin(Att(2)) 0  cos(Att(2))];
            Rpsi = [ cos(Att(3)) sin(Att(3)) 0
                    -sin(Att(3)) cos(Att(3)) 0
                     0         0         1];
            Reb = Rphi*Rtheta*Rpsi;
            Rbe = Reb';

            % Translate
            T = [eye(3)     -Pos
                 zeros(1,3)  1];
            V = [V; ones(1,size(V,2))];
            V = (T*V);
            V = V(1:3,:);

            % Rotate
            V = (Reb*V);

        end

        % Transform positions and vertices to Earth axes
        function V = TransformtoEarthAxes(V,Pos,Att)

            % Rotation matrices
            Rphi = [1  0         0
                    0  cos(Att(1)) sin(Att(1))
                    0 -sin(Att(1)) cos(Att(1))];
            Rtheta = [cos(Att(2)) 0 -sin(Att(2))
                      0         1  0
                      sin(Att(2)) 0  cos(Att(2))];
            Rpsi = [ cos(Att(3)) sin(Att(3)) 0
                    -sin(Att(3)) cos(Att(3)) 0
                     0         0         1];
            Reb = Rphi*Rtheta*Rpsi;
            Rbe = Reb';

            % Transformation
            S = [Rbe Pos
                 zeros(1,3) 1];
            V = S*[V' 1]';
            V = V(1:3);

        end

    end

end
