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
        SearchType = "Smart";
        InitialSearchType = "Smart";
        BatteryUsage = 0;
        FailureType = "N/A"
        QuadInitialized = 0;
        SystemsOkay
        MissionComplete
        MissionFailed = 0;
        MissionEnd = 0;
        AbortTime = 300;
        ModeEntryTime = 0;
        Waypoints
        IDPosition;
        WP = 1;
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
        P_SystemsFault = 0.05;
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
                obj.Decisions.CurrentStates = 303;
                obj.Decisions.States = varargin{i1+1};
                obj.Decisions.Transitions = varargin{i2+1};
%                 obj.Decisions.State = 303;
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
