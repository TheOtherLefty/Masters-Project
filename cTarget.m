classdef cTarget < cAgent
    %TARGET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        Shape;
        Position = [1 0 0]';
        Size = [0.2 0.2 0.2]';
        Angle = 10*pi/180;
        Vertices
        Faces
        Colours
        
        % Object properties
        m = 0.4;      % Mass (kg)
        g = 9.81;   % Acceleration due to gravity (m/s2)
        Ix = 0.01;   % Inertia about x-axis (kg m2)
        Iy = 0.01;   % Inertia about y-axis (kg m2)
        Iz = 0.01;   % Inertia about z-axis (kg m2)
        
        % Floor properties
        FloorNatFreq = 50;
        FloorDamp = 0.5;
        
        % Grabber properties
        TargetGrabbed = 0;
        GrabEntry = 1;
        
    end
    
    methods
        
        % CLASS CONSTRUCTOR
        function obj = cTarget(ID,Environment,varargin)
            
            % ENVIRONMENT PROPERTIES
            obj.RoomLimits = Environment.RoomLimits;
            
            obj.ID = ID;
            obj.Shape = ID;
            
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
            obj.EffRadius = 0.05;
            i = find(strcmp(varargin,'Pose'));
            if strcmp(varargin{i+1},'random')
                Pose = obj.InitPosition;
            elseif ~isempty(i)
                Pose = varargin{i+1};
            else
                error('Please enter an initial position for the quadrotor or use ''random''')
            end
            
            % Intialise states
            obj.States = [Pose; zeros(6,1)];
            
            % Initialise geometry
            obj.Geometry = obj.InitialiseGeometry;
            obj.Geometry = obj.UpdateGeometry(obj.Geometry,obj.States);
            
            % Set integration algorithm
            i = find(strcmp(varargin,'Solver'));
            if isempty(i)
                obj.Solver = 'Euler';
            else
                obj.Solver = varargin{i+1};
            end
            
        end
        
        % UPDATE AGENT
        function BB = UpdateAgent(obj,BB,ind)
            
            % Set agent index
            if isempty(obj.Index)
                obj.Index = ind;
            end
            
            % Load grabber properties
            for a = 1:length(BB.Agents.Quad)
                if ~isempty(BB.Agents.Quad(a).GrabbedTargets)
                    GrabbingQuads(a) = BB.Agents.Quad(a).GrabbedTargets(ind);
                    QuadInd = a;
                else
                    GrabbingQuads(a) = 0;
                end
            end
            
            obj.TargetGrabbed = any(GrabbingQuads);
            
            % Update states
            if obj.TargetGrabbed
                
                % Yaw difference between quad and target
                dpsi = BB.Agents.Quad(QuadInd).States(6) - obj.States(6);
                Rpsi = [cos(dpsi) -sin(dpsi) 0
                        sin(dpsi)  cos(dpsi) 0
                        0          0         1];
                
                obj.StateTrans = [BB.Agents.Quad(QuadInd).StateTrans(16:18)
                    Rpsi*BB.Agents.Quad(QuadInd).StateTrans(4:6)
                    BB.Agents.Quad(QuadInd).StateTrans(19:21)
                    Rpsi*BB.Agents.Quad(QuadInd).StateTrans(10:12)];
                
                obj.States = obj.States + obj.TimeStep*obj.StateTrans;
                obj.Time = obj.Time + obj.TimeStep;
            else
                [obj.States,obj.StateTrans,obj.Time] = ...
                    obj.Integrate(obj.States,obj.Inputs,obj.Time);
            end
            
            % Update geometry
            obj.Geometry = obj.UpdateGeometry(obj.Geometry,obj.States);
            
            % Update blackboard
            Props = {'Time','States'};
            for p = 1:length(Props)
                BB.Agents.Target(ind).(Props{p}) = obj.(Props{p});
            end
            
            % Update geometry
            ID = obj.Geometry.ID;
            BB.Geometry.Vertices(ID:ID+size(obj.Geometry.Vertices,1)-1,:)...
                = obj.Geometry.Vertices;
            
        end
        
        % OBJECT DYNAMICS
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

            % Inertias
            I = diag([obj.Ix obj.Iy obj.Iz]);
            
            % GROUND MODEL
            
            % Floor spring and damping
            kf = obj.m*obj.FloorNatFreq^2;
            cf = obj.m*2*obj.FloorDamp*obj.FloorNatFreq;
            
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
            F = G;
            
            % Moments
            M = Gm;
            
            % STATE DERIVATIVES
            Xdot = [X(7:9)
                    X(10:12)
                    Rbe*F/obj.m + [0 0 obj.g]';
                    I\M];
            
        end
        
        % INITIALISE GEOMETRY
        function Geo = InitialiseGeometry(obj)
            
            switch obj.Shape
                case 'Ball'
                    
                    % Radius
                    R = 0.05;
                    obj.EffRadius = R;
                    
                    [X,Y,Z] = sphere(10);
                    fvc = surf2patch(R*X,R*Y,R*Z);
                    C = [1 0 0];
                    C = repmat(C,size(fvc.faces,1),1);
                    A = ones(size(fvc.faces,1),1);

                    Geo.Faces = fvc.faces;
                    Geo.Vertices = fvc.vertices;
                    Geo.BodyVertices = fvc.vertices;
                    Geo.Colours = C;
                    Geo.Alpha = A;
                    
                case 'Cube'
                    
                    % Dimensions
                    l = 0.2;
                    w = 0.1;
                    h = 0.1;
                    obj.EffRadius = h/2;
                    
                    V = [ l/2  w/2  h/2
                         -l/2  w/2  h/2
                         -l/2 -w/2  h/2
                          l/2 -w/2  h/2
                          l/2  w/2 -h/2
                         -l/2  w/2 -h/2
                         -l/2 -w/2 -h/2
                          l/2 -w/2 -h/2];
                    F = [1 2 3 4
                         5 6 7 8
                         1 2 6 5
                         2 3 7 6
                         3 4 8 7
                         4 1 5 8];
                    C = [0 1 0];
                    C = repmat(C,size(F,1),1);
                    A = ones(size(F,1),1);
                    
                    Geo.Faces = F;
                    Geo.Vertices = V;
                    Geo.BodyVertices = V;
                    Geo.Colours = C;
                    Geo.Alpha = A;
                    
                case 'Pyramid'
                    
                    % Dimensions
                    w = 0.2;
                    h = 0.15;
                    obj.EffRadius = h/2;
                    
                    V = [ w/2  w/2  h/2
                         -w/2  w/2  h/2
                         -w/2 -w/2  h/2
                          w/2 -w/2  h/2
                          0    0   -h/2];
                    F = [1 2 3 4
                         1 2 5 NaN
                         2 3 5 NaN
                         3 4 5 NaN
                         4 1 5 NaN];
                    C = [0 0 1];
                    C = repmat(C,size(F,1),1);
                    A = ones(size(F,1),1);
                    
                    Geo.Faces = F;
                    Geo.Vertices = V;
                    Geo.BodyVertices = V;
                    Geo.Colours = C;
                    Geo.Alpha = A;
                    
                    
            end
            
        end
        
    end
    
end

