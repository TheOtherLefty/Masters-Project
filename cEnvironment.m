classdef cEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        ID
        CellSize = 0.5
        RoomSize
        GridSize
        Geometry
        
    end
    
    methods
        
        function obj = cEnvironment(varargin)
            
            obj.ID = 'Lab';
            
            % Get drop location if specified
            if nargin > 0
                
                % Grid size
                i = find(strcmp(varargin,'Grid size'));
                if isempty(i)
                    obj.GridSize = [5, 5]';
                else
                    obj.GridSize = (varargin{i+1}(1:2))';
                end
                
                % Drop site
                i = find(strcmp(varargin,'Dropsite'));
                if isempty(i)
                    obj.Geometry.DropLocation = [-0.25 -0.25 0]';
                else
                    obj.Geometry.DropLocation = varargin{i+1};
                end
                
            else
                
                obj.Geometry.DropLocation = [0 0 0]';
                obj.GridSize = [5, 5]';
                
            end
            
            % Set room limits
            obj.RoomSize = [obj.CellSize*(obj.GridSize(1:2)'-1) -2]';
            
            % Initialise geometry
            [obj.Geometry.Vertices,obj.Geometry.Faces,...
                obj.Geometry.Colours,obj.Geometry.Alpha]...
                = obj.InitialiseGeometry;
            
            obj.Geometry.DropRadius = 0.6;
            obj.Geometry.DropSite = obj.InitialiseDropSite;
            
        end
        
        function [V,F,C,A] = InitialiseGeometry(obj)
            
            % Mid-point of room
            r = obj.RoomSize/2;
            
            % Initial shape
            V0 = [ r(1)+0.5  r(2)+0.5  r(3)
                  -r(1)-0.5  r(2)+0.5  r(3)
                  -r(1)-0.5 -r(2)-0.5  r(3)
                   r(1)+0.5 -r(2)-0.5  r(3)
                   r(1)+0.5  r(2)+0.5 -r(3)
                  -r(1)-0.5  r(2)+0.5 -r(3)
                  -r(1)-0.5 -r(2)-0.5 -r(3)
                   r(1)+0.5 -r(2)-0.5 -r(3)];
               
            % Translate and rotate
            V = [V0 ones(size(V0,1),1)];
            T = [eye(3) [0, 0, -1]';
                 zeros(1,3) 1];
            V = (T*V')'; 
            V = V(:,1:3);
            
            % Faces
            F = [5 6 7 8];
            
            % Colour
            c = [1 1 1];
            C = repmat(c,size(F,1),1);
            
            % Alpha
            a = 0;
            A = repmat(a,size(F,1),1);
            
        end
        
        function DS = InitialiseDropSite(obj)
            
            R = obj.Geometry.DropRadius;
            P = obj.Geometry.DropLocation;
            
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

