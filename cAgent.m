classdef cAgent < handle
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % Agent properties
        ID;
        Index;
        Time = 0;
        TimeFinal;
        TimeStep;
        States;
        StateTrans;
        StatesEst;
        Inputs;
        PseudoInputs;
        Outputs;
        Commands;
        Mode;
        Status;
        PrevMode;
        ModeTransition = 'Active';
        Solver = 'Euler';
        Geometry;
        RoomSize;
        GridSize;
        CellSize;
        EffRadius;
        BatteryLevel = 0;
        
    end
    
    methods
        
        % CLASS CONSTRUCTOR
%         function obj = cAgent
            
            
            
%         end
        
        % INITIALISE POSITION
        function [Pose] = InitPosition(obj)
            
            
            % Version 1
%             Pose = [(obj.RoomLimits(1:2,2)-obj.RoomLimits(1:2,1)-0.2).*(rand(2,1)-0.5)...
%                 + (obj.RoomLimits(1:2,2)+obj.RoomLimits(1:2,1))/2
%                     -obj.EffRadius
%                     zeros(2,1)
%                     0*wrapToPi(2*pi*rand)];

            % Version 2
%            Pose = [(obj.RoomSize(1)+obj.CellSize)*rand - obj.CellSize/2
%                    (obj.RoomSize(2)+obj.CellSize)*rand - obj.CellSize/2
%                    -obj.EffRadius
%                    zeros(2,1)
%                    0*wrapToPi(2*pi*rand)];
                
            % 2/2019
            % Version 3: Objects are randomly allocated an integer
            % gridpoint which is then scaled to the 0.5m size of each
            % gridpoint. Objects then have a 0.5m variance within that
            % gridpoint.
            
            % InitCoords retains a list of the integer gridpoints which
            % already have objects at them. genCoords is repeatedly
            % randomly assigned coordinates until an unoccupied coordinate
            % is used.

            persistent InitCoords;
            
            genCoords = [randi([0,(obj.RoomSize(1)-0.5)*2])-(obj.RoomSize(1)),randi([0,(obj.RoomSize(2)-0.5)*2])-(obj.RoomSize(2))];
            
            if ~isempty(InitCoords) %ismember(~,~,'rows') fails on empty matrice
                while ismember(genCoords, InitCoords, 'rows') || (genCoords(1) >= -1 && genCoords(1) <= 1) && (genCoords(2) >= -1 && genCoords(2) <= 1)
                    genCoords = [randi([0,(obj.RoomSize(1)-0.5)*2])-(obj.RoomSize(1)),randi([0,(obj.RoomSize(2)-0.5)*2])-(obj.RoomSize(2))];
                end
            else
                while (genCoords(1) >= -1 && genCoords(1) <= 1) && (genCoords(2) >= -1 && genCoords(2) <= 1) % Still need to check for objects spawning at base locations
                    genCoords = [randi([0,(obj.RoomSize(1)-0.5)*2])-(obj.RoomSize(1)),randi([0,(obj.RoomSize(2)-0.5)*2])-(obj.RoomSize(2))];
                end
            end
            
            InitCoords = [InitCoords; genCoords]
            
            Pose = [genCoords(1)*0.5+(rand-0.5)*0.5
                    genCoords(2)*0.5+(rand-0.5)*0.5
                    -obj.EffRadius
                    zeros(2,1)
                    0*wrapToPi(2*pi*rand)];                
        end
        
        % INTEGRATE STATE DERIVATIVES
        function [X,Xdot,t] = Integrate(obj,X,U,t)
            
            dt = obj.TimeStep;
            
            switch obj.Solver
                case 'Euler'
                    
                    Xdot = obj.Dynamics(X,U,t);
                    X = X + Xdot*dt;
                    t = round(t/dt + 1)*dt;
                    
                case 'RK4'
                    
                    k1 = obj.Dynamics(X,U,t);
                    Xdot = k1;
                    k2 = obj.Dynamics(X+dt*k1/2,U,t+dt/2);
                    k3 = obj.Dynamics(X+dt*k2/2,U,t+dt/2);
                    k4 = obj.Dynamics(X+dt*k3,U,t+dt);
                    X = X + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
                    t = round(t/dt + 1)*dt;
                    
            end
            
        end
        
        % UPDATE GEOMETRY
        function Geo = UpdateGeometry(obj,Geo,X)
            
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
            
            % Transformation matrix
            T = [Rbe        X(1:3)
                 zeros(1,3) 1];
            
            % Rotate and translate in World frame
            Vb = [Geo.BodyVertices ones(size(Geo.BodyVertices,1),1)];
            V = (T*Vb')';
            Geo.Vertices = V(:,1:3);
            
        end
        
    end
    
end

