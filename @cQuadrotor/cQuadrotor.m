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
        mO = 0*0.1;

        % Floor properties
        FloorNatFreq = 50;
        FloorDamp = 0.5;

        % Power properties
        MaxBattery = 34;
        ChargeRate = 0.025; % Old
        LossRate = -0.005;  % Old
        BatteryRate;        % Old
        WarningLevel = 10.55;   % Old
        BatteryLossRate = 2;

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
        dt_Controller = 0.002;
        VelFilter = 50;
        VelInt = [0 0 0]';
        Cint = 0;
        ActuatorLimits = 1;
        epos_int = [0 0 0]';

        % Sensor properties
        dt_Sensor = 0.002;
        CameraPosition = [0 0 0.1]';
        CameraAngle = -90*pi/180;
        Camera
        n_Acc = 1e-2;
        n_Gyro = 1e-3;
        n_OT = 1e-4;

        % Mission properties
        SearchedQuadrants = 0;
        SearchType = "Smart";
        InitialSearchType = "Smart";
        BatteryUsage = 0;
        FailureType = "N/A"
        QuadInitialized = 0;
        SystemsOkay
        MissionComplete
        MissionFailed = 0;
        MissionEnd = 0;
        AbortTime = 600;
        ModeEntryTime = 0;
        Waypoints
        IDPosition;
        WP = 56;
        WPcount = 1;
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
        Decisions

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
        P_SystemsFault = 0.0005;
        ActuatorFaults = [0 0 0 0]';

    end

    methods

        % CLASS CONSTRUCTOR -----------------------------------------------
        function obj = cQuadrotor(ID,Environment,varargin)
            
            % ENVIRONMENT PROPERTIES
            obj.RoomSize = Environment.RoomSize;
            obj.GridSize = Environment.GridSize;
            obj.CellSize = Environment.CellSize;

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
            
            obj.dt_Controller = obj.TimeStep;
            obj.dt_Sensor = obj.TimeStep;

            % Initialise position
            obj.EffRadius = 0.2;
            i = find(strcmp(varargin,'Pose'));
            if isempty(i)
                Pose = obj.InitPosition;
            else
                Pose = varargin{i+1};
            end
            
            % Initialise autonomous decisions
            i1 = find(strcmp(varargin,'States'));
            i2 = find(strcmp(varargin,'Transitions'));
            if isempty(i1) || isempty(i2)
                obj.Decisions = [];
            else
                obj.Decisions.CurrentStates = 301;
                obj.Decisions.States = varargin{i1+1};
                obj.Decisions.Transitions = varargin{i2+1};
                obj.Decisions.Gridpoints = ones(1,prod(obj.GridSize));
                obj.Decisions.Mode = 0;
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
            obj.BatteryLevel = obj.MaxBattery;

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
            obj.Camera.BndRadius = 100;
            obj.Camera.Vertices = [];
            obj.Camera.SceneVertices = [];
            obj.Camera.DropSite = [];
            obj.Camera.FocalLength = obj.Camera.Res(1)/(2*tan(obj.Camera.FOV));
            obj.Camera.Bnd = obj.Camera.FocalLength*0.5/1;
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

    end
    
    methods
        
        function [nx,ny]= adjustCoords(obj, x, y)
            switch nearestBaseWP(obj)
                case 55
                    nx = x+1;
                    ny = y;
                case 46
                    ny = y+1;
                    nx = x;
                case 45
                    nx = x+1;
                    ny = y+1;
                case 56
                    nx = x;
                    ny = y;
            end
        end
           
        function nextWP = transitionController(obj)
            % Select new controller file based on how many quadrants have
            % been searched, this tells us which quadrant we must be in.
            switch obj.SearchedQuadrants
                case 0
                    nextWP = 56;
                    DecisionsFile = 'scenario3b_5x5_1';
                case 1
                    % Moves the quadrotor to the next quadrant
                    % (specifically the base location in the next quadrant.
                    nextWP = 55;
                    
                    % Different controller files according to how many
                    % objects are left to be found.
                    switch obj.TargetCount
                        case 0
                            DecisionsFile = 'scenario3b_5x5_2_3'
                        case 1
                            DecisionsFile = 'scenario3b_5x5_2_2'
                        case 2
                            DecisionsFile = 'scenario3b_5x5_2_1'
                    end
                case 2
                    nextWP = 45;
                    switch obj.TargetCount
                        case 0
                            DecisionsFile = 'scenario3b_5x5_3_3'
                        case 1
                            DecisionsFile = 'scenario3b_5x5_3_2'
                        case 2
                            DecisionsFile = 'scenario3b_5x5_3_1'
                    end
                case 3
                    nextWP = 46;
                    switch obj.TargetCount
                        case 0
                            DecisionsFile = 'scenario3b_5x5_4_3'
                        case 1
                            DecisionsFile = 'scenario3b_5x5_4_2'
                        case 2
                            DecisionsFile = 'scenario3b_5x5_4_1'
                    end
                case 4
                    % In practice this will almost never be called as the
                    % controller will usually realise something is wrong before it
                    % reached the base where it would call this. Instead
                    % this fail state is handles in SmartSearch.m
                    obj.MissionFailed = 1;
                    obj.FailureType = "Searched full grid: still missing objects.";
            end
            
            % Update controller states and transition tables to match those
            % stored in the new state and transition files.
            [obj.Decisions.States, obj.Decisions.Transitions] = LoadDecisions(DecisionsFile);
        end
        
        % Given a direction from a controller file, determines the actual
        % direction the quadrotor should move in on the environment grid.
        function tDirection = translateDirection(obj, direction)
            
            switch nearestBaseWP(obj)
                case 56
                    tDirection = direction;
                case 55 
                    if strcmp(direction,'east') || strcmp(direction,'west')
                        tDirection = cQuadrotor.reverseDirection(direction);
                    else
                        tDirection = direction;
                    end
                case 46
                    if strcmp(direction,'north') || strcmp(direction,'south')
                        tDirection = cQuadrotor.reverseDirection(direction);
                    else 
                        tDirection = direction;
                    end
                case 45
                    tDirection = cQuadrotor.reverseDirection(direction);
            end
        end
        
        function globalWP = getGlobalWP(obj, quadrantGP)
            % Locates a quadrant gridpoint in the global list of
            % gridpoints. Used for waypoint setting as Waypoints must be
            % global.
            
            switch nearestBaseWP(obj)
                case 56
                    globalWP = 56+mod(quadrantGP-1,5)+floor(quadrantGP-1/5)*10;
                case 55
                    globalWP = 55-mod(quadrantGP-1,5)+floor(quadrantGP-1/5)*10;                  
                case 45
                    globalWP = 45-mod(quadrantGP-1,5)-floor(quadrantGP-1/5)*10;
                case 46
                    globalWP = 46+mod(quadrantGP-1,5)-floor(quadrantGP-1/5)*10;
            end
        end
        
        function obj = updateGPS(obj, quadrantGP, value)
            % Updates a quadrant gridpoints searched status in the list of
            % global gridpoints.
            
            obj.Decisions.Gridpoints(getGlobalWP(obj)) = value;
        end
        
        function QuadrantGPS = translateGPS(obj)
            % Returns the searched values of the gridpoints in the quadrant
            % that the UAV is currently in. In the format of the PRISM
            % controller; i.e. gps = [1:25]. obj.Decisions.Gridpoints
            % contains the searched values for all the gridpoints in all
            % four quadrants so they need to be converted.
            
            if mod(obj.WP-1,10) < 5
                if obj.WP > 50
                    QuadrantGPS = obj.Decisions.Gridpoints([91:95 81:85 71:75 61:65 51:55]);
                else
                    QuadrantGPS = obj.Decisions.Gridpoints([1:5 11:15 21:25 31:35 41:45]);
                end
            else
                if obj.WP > 50
                    QuadrantGPS = obj.Decisions.Gridpoints([100:-1:96 90:-1:86 80:-1:76 70:-1:66 60:-1:56]);
                else
                    QuadrantGPS = obj.Decisions.Gridpoints([10:-1:6 20:-1:16 30:-1:26 40:-1:36 50:-1:46]);
                end
            end
            
            QuadrantGPS = QuadrantGPS(end:-1:1);
        end
        
        function bool = atBase(obj)
            % returns true if the quad is at a base grid point, false
            % otherwise.
            if (obj.WP == 45) || (obj.WP == 46) || (obj.WP == 55) || (obj.WP == 56)
                bool = true;
            else
                bool = false;
            end 
        end
        
        function BaseWP = nearestBaseWP(obj)
            % for a 10/10 grid, determines the closes base WP given the
            % current WP of the quad.
            if mod(obj.WP-1,10) < 5
                if obj.WP > 50
                    BaseWP = 55;
                else
                    BaseWP = 45;
                end
            else
                if obj.WP > 50
                    BaseWP = 56;
                else
                    BaseWP = 46;
                end
            end
        end
    end
    

    methods(Static)
        function rDirection = reverseDirection(direction)
            if strcmp(direction,"")
                rDirection = direction;
            else
                switch direction
                    case 'north'
                        rDirection = 'south';
                    case 'south'
                        rDirection = 'north';
                    case 'east'
                        rDirection = 'west';
                    case 'west'
                        rDirection = 'east';
                    otherwise
                        rDirection = direction;
                end
            end
        end

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
