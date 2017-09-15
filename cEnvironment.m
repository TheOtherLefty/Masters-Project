classdef cEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        ID
        Size = [4 7 3]';
        Geometry
        RoomLimits
        
    end
    
    methods
        
        function obj = cEnvironment(varargin)
            
            obj.ID = 'Lab';
            
            [obj.Geometry.Vertices,obj.Geometry.Faces,...
                obj.Geometry.Colours,obj.Geometry.Alpha]...
                = obj.InitialiseGeometry;
            
            % Set room limits
            obj.RoomLimits = [min(obj.Geometry.Vertices)
                              max(obj.Geometry.Vertices)]';
            
            % Get drop location if specified
            i = find(strcmp(varargin,'Dropsite'));
            if ~isempty(i)
                obj.Geometry.DropLocation = varargin{i+1};
            else
                obj.Geometry.DropLocation = [];
            end
            obj.Geometry.DropRadius = 0.25;
            obj.Geometry.DropSite = obj.InitialiseDropSite;
            
        end
        
        function [V,F,C,A] = InitialiseGeometry(obj)
            
            % Set height as ground
            r = [0 0 -obj.Size(3)/2]';
            
            % Initial shape
            V0 = [ obj.Size(1)/2  obj.Size(2)/2  obj.Size(3)/2
                  -obj.Size(1)/2  obj.Size(2)/2  obj.Size(3)/2
                  -obj.Size(1)/2 -obj.Size(2)/2  obj.Size(3)/2
                   obj.Size(1)/2 -obj.Size(2)/2  obj.Size(3)/2
                   obj.Size(1)/2  obj.Size(2)/2 -obj.Size(3)/2
                  -obj.Size(1)/2  obj.Size(2)/2 -obj.Size(3)/2
                  -obj.Size(1)/2 -obj.Size(2)/2 -obj.Size(3)/2
                   obj.Size(1)/2 -obj.Size(2)/2 -obj.Size(3)/2];
               
            % Translate and rotate
            V = [V0 ones(size(V0,1),1)];
            T = [eye(3) r
                 zeros(1,3) 1];
            V = (T*V')'; 
            V = V(:,1:3);
            
            % Faces
            F = [1 2 3 4];
            
            % Colour
            c = [1 1 1];
            C = repmat(c,size(F,1),1);
            
            % Alpha
            a = 0;
            A = repmat(a,size(F,1),1);
            
        end
        
        function DS = InitialiseDropSite(obj)
            
            R = obj.Geometry.DropRadius;
            P = [(obj.RoomLimits(1:2,2)-obj.RoomLimits(1:2,1)-2*R).*(rand(2,1)-0.5)...
                + (obj.RoomLimits(1:2,2)+obj.RoomLimits(1:2,1))/2
                0];
            if isempty(obj.Geometry.DropLocation)
                obj.Geometry.DropLocation = P;
            else
                P = obj.Geometry.DropLocation;
            end
            
            % Number of sides of circle
            N = 16;
            psi = 0:2*pi/N:2*pi;
            
            C = [R*cos(psi)
                 R*sin(psi)
                 0*psi];
            S = [eye(3) P
                 zeros(1,3) 1];
            DS = S*[C; ones(1,size(C,2))];
            DS = DS(1:3,:);
            
        end
        
    end
    
end

