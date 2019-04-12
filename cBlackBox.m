classdef cBlackBox < handle
    %BLACKBOX Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % Blackbox data
        tsamp
        
        % Agent data
        Time
        Agents
        AgentTypes
        NoAgentTypes
        AgentTypeNos
        
        % Environment data
        Environment
        Geometry
        
        % Output properties
        DimSmall = [600 400];
        DimLarge = [1400 900];
        DimVideo = [1024 768];
        SaveVideo = 0;
        
    end
    
    methods
        
        % CLASS CONSTRUCTOR -----------------------------------------------
        function obj = cBlackBox
            
            % Font and formatting
            set(0,'DefaultAxesFontName','Times')
%             set(0,'DefaultAxesMarkerOrder','o','.','*','s')
            
        end
        
        % INITIALISE BLACK BOX --------------------------------------------
        function Initialise(obj,Blackboard)
            
            obj.Time = Blackboard.GlobalTime;
            obj.Environment = Blackboard.Environment;
            obj.Agents = Blackboard.Agents;
            obj.Geometry = Blackboard.Geometry;
            obj.AgentTypes = fieldnames(obj.Agents);
            obj.NoAgentTypes = length(obj.AgentTypes);
            for i = 1:obj.NoAgentTypes
                obj.AgentTypeNos(i) = length(obj.Agents.(obj.AgentTypes{i}));
            end
            
            % Fix geometry
            obj.Geometry.Vertices = {obj.Geometry.Vertices};
%             obj.Geometry.CameraVertices = {obj.Geometry.CameraVertices};
            
            % Fix modes and geometry
            for j = 1:obj.NoAgentTypes
                Type = obj.AgentTypes{j};
                for i = 1:obj.AgentTypeNos(j)
                    obj.Agents.(Type)(i).Mode = {obj.Agents.(Type)(i).Mode};
                    obj.Agents.(Type)(i).Status = {obj.Agents.(Type)(i).Status};
                    if strcmp(Type,'Quad')
                        obj.Agents.(Type)(i).Camera.Vertices = {obj.Agents.(Type)(i).Camera.Vertices};
                        obj.Agents.(Type)(i).Camera.Centroid = {obj.Agents.(Type)(i).Camera.Centroid};
                        obj.Agents.(Type)(i).Camera.DropSite = {obj.Agents.(Type)(i).Camera.DropSite};
                    end
                end
            end
        end
        
        % UPDATE BLACK BOX ------------------------------------------------
        function Update(obj,Blackboard)
            
            % Update time
            obj.Time(end+1) = Blackboard.GlobalTime;
            
            % Loop agent types
            for j = 1:obj.NoAgentTypes
                
                % Current type
                Type = obj.AgentTypes{j};
                
                % Loop agents of type
                for i = 1:length(obj.Agents.(Type))
                    
                    Props = {'Time','States','Inputs','PseudoInputs','Outputs','Commands'};
                    for p = 1:length(Props)
                        if ~isempty(Blackboard.Agents.(Type)(i).(Props{p}))
                            obj.Agents.(Type)(i).(Props{p})(:,end+1)...
                                = Blackboard.Agents.(Type)(i).(Props{p});
                        end
                    end
                    
                    if strcmp(Type,'Quad')
                        obj.Agents.(Type)(i).Camera.Vertices{:,:,end+1}...
                            = Blackboard.Agents.(Type)(i).Camera.Vertices;
                        obj.Agents.(Type)(i).Camera.Centroid{end+1}...
                            = Blackboard.Agents.(Type)(i).Camera.Centroid;
                        obj.Agents.(Type)(i).Camera.DropSite{end+1}...
                            = Blackboard.Agents.(Type)(i).Camera.DropSite;
                        obj.Agents.(Type)(i).BatteryLevel(end+1)...
                            = Blackboard.Agents.(Type)(i).BatteryLevel;
                        obj.Agents.(Type)(i).TargetCount(end+1)...
                            = Blackboard.Agents.(Type)(i).TargetCount;
                    end
                    
                    % Update mode
                    obj.Agents.(Type)(i).Mode{end+1}...
                        = Blackboard.Agents.(Type)(i).Mode;
                    
                    % Update status
                    obj.Agents.(Type)(i).Status{end+1}...
                        = Blackboard.Agents.(Type)(i).Status;
                    
                end
                
            end
                    
            % Update geometry
            obj.Geometry.Vertices{:,:,end+1} = Blackboard.Geometry.Vertices;
%             obj.Geometry.CameraVertices{:,:,end+1} = Blackboard.Geometry.CameraVertices;
            
        end
        
        % PLOT FUNCTION ---------------------------------------------------
        function PlotData(obj)
            
            options = {'1: Animate agents'
                       '2: Compare states'
                       '3: Compare inputs'
                       '0: Exit'
                       'x: Exit and close all windows'};

            viz = '1';
            
            % Loop user selection
            while ~strcmp(viz,'0') && ~strcmp(viz,'x')

                % List visualisation options
                fprintf('\n+- VISUALISATIONS -------------------------\n|\n')
                for i = 1:length(options)
                    fprintf('|  %s  \n',options{i})
                end
                fprintf('|\n')
                
                % Ask user for visualisation choice
                viz = input('|  Select visualisation: ','s');
                
                fprintf('|\n+------------------------------------------\n\n')
                
                % Select visualisation
                obj.PlotSelection(viz);
                
                % Close windows if option selected
                if strcmp(viz,'x')
                    close all
                end
                
            end
            
        end
        
        % PLOT SELECTION --------------------------------------------------
        function PlotSelection(obj,viz)
            
            switch viz
                case '1'
                    fprintf('Animating agents...\n\n')
                    obj.Animate;
                case '2'
                    fprintf('Generating state comparison...\n\n')
                    obj.PlotStates;
                case '3'
                    fprintf('Generating input comparison...\n\n')
                    obj.PlotInputs;
                case {'0','x'}
                    fprintf('Exiting...\n\n')
                otherwise
                    fprintf('Invalid selection, please try again\n\n')
            end
            
        end
        
        % ANIMATE AGENTS --------------------------------------------------
        function Animate(obj)
            
            % Initialise theatre
            f = figure('Name','Agent animation','Position',obj.CentreFig(obj.DimLarge),...
                'Renderer','OpenGL','Color','w');
%                 'Menubar','none','Toolbar','none');
            if obj.SaveVideo
                set(f,'Position',obj.CentreFig(obj.DimVideo));
                set(0,'DefaultAxesFontName','Raleway Medium')
            end
            
            % 3D axes
            a1 = axes;
            xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
            set(a1,'xdir','reverse','zdir','reverse','Clipping','off');
            rotate3d, axis equal, grid on
            view(3), box on, hold on
            shading flat
            lighting flat
            light('Position',-5*[1 1 1])
            
            % Set limits
            xlim(obj.Geometry.Limits(1,:))
            ylim(obj.Geometry.Limits(2,:))
            zlim(obj.Geometry.Limits(3,:))
            
            % Camera axes
            a2 = axes('Position',[0.1 0.12 0.2 0.2]);
            xlabel('x'), ylabel('y')
            set(a2,'ydir','reverse','Clipping','on');
            rotate3d, axis equal
            view([0 0 -1]), box on, hold on, grid on
            Res = obj.Agents.Quad(1).Camera.Res;
            Bnd = obj.Agents.Quad(1).Camera.Bnd;
            xlim(Res(1)*0.5*[-1 1])
            ylim(Res(2)*0.5*[-1 1])
            zlim([10 1000])
            plot3([-Bnd/2 -Bnd/2 Bnd/2 Bnd/2 -Bnd/2],...
                [-Bnd/2 Bnd/2 Bnd/2 -Bnd/2 -Bnd/2],...
                [10 10 10 10 10],'r--')
%             NBnd = 16;
%             psiBnd = (0:NBnd)*2*pi/NBnd;
%             plot3(Bnd*cos(psiBnd),Bnd*sin(psiBnd),0*psiBnd+10,'r--')
%             plot3([0 0],0.5*Res(2)*[-1 1],[10 10],'k-.')
%             plot3(0.5*Res(1)*[-1 1],[0 0],[10 10],'k-.')
            
            % Initialise video
            if obj.SaveVideo
                mov = VideoWriter('Outputs/Animation','MPEG-4');
                open(mov);
            end
            
            % Patch objects
            axes(a1)
            p = patch('Faces',obj.Geometry.Faces,...
                'Vertices',obj.Geometry.Vertices{:,:,1},...
                'FaceVertexCData',obj.Geometry.Colours,...
                'FaceVertexAlphaData',obj.Geometry.Alpha,...
                'FaceColor','flat','CDataMapping','direct',...
                'FaceAlpha','flat','AlphaDataMapping','none',...
                'DiffuseStrength',0.9,'AmbientStrength',0.5);
            rotate3d
            
            % Flight path
            pt = plot3(0,0,0,'b');
            
            % Dropsite
            pds = plot3(obj.Geometry.DropSite(1,:),...
                obj.Geometry.DropSite(2,:),...
                obj.Geometry.DropSite(3,:),'r','LineWidth',2);
            
            % Perspective mode
            set(a1,'Projection','Perspective')
            
            % Show time
            at = axes('Position',[0.1 0.9 0.2 0.1]);
%             at = axes('Position',[0.2 0.7 0.2 0.1]);
            axis off
            
            if obj.SaveVideo
                FontName = 'Raleway Bold';
                FontSize = 10;
            else
                FontName = 'Times';
                FontSize = 12;
            end
            
            tt = text(0,0.5,'Time: 0.00s','FontName',FontName,'FontSize',FontSize);
            
            % Show mode
            if obj.AgentTypeNos(ismember(obj.AgentTypes,'Quad')) == 1
                tm = text(0,0,'Mode: Idle','FontName',FontName,'FontSize',FontSize);
            end
            
            % Animate camera
            axes(a2)
            pc = patch('Faces',[],'Vertices',[],'FaceVertexCData',[],...
                'FaceVertexAlphaData',[],'FaceColor','flat','CDataMapping','direct',...
                'FaceAlpha','flat','AlphaDataMapping','none',...
                'DiffuseStrength',0.9,'AmbientStrength',0.5);
            pct(1) = plot3(NaN,NaN,NaN,'y+','MarkerSize',15,'LineWidth',2);
            pct(2) = plot3(NaN,NaN,NaN,'bo','MarkerSize',15,'LineWidth',2);
            pcds = plot3(NaN,NaN,NaN,'r','LineWidth',2);
            shading flat
            
            % Animate agents
            for k = 1:2:length(obj.Time)
                
                set(p,'Vertices',obj.Geometry.Vertices{:,:,k})
                
                set(pt,'xdata',obj.Agents.Quad(1).States(1,1:k),...
                    'ydata',obj.Agents.Quad(1).States(2,1:k),...
                    'zdata',obj.Agents.Quad(1).States(3,1:k))
                
                set(tt,'String',['Time: ',num2str(obj.Time(k),'%.2f'),'s'])
                
                if obj.AgentTypeNos(ismember(obj.AgentTypes,'Quad')) == 1
                    set(tm,'String',['Mode: ',obj.Agents.Quad(1).Mode{k}])
                end
                
                if isempty(obj.Agents.Quad(1).Camera.Vertices{:,:,k})
                    set(pc,'Faces',[],'Vertices',[],'FaceVertexCData',[],...
                        'FaceVertexAlphaData',[])
                else
                    set(pc,'Faces',obj.Geometry.Faces,...
                        'Vertices',obj.Agents.Quad(1).Camera.Vertices{:,:,k},...
                        'FaceVertexCData',obj.Geometry.Colours,...
                        'FaceVertexAlphaData',obj.Geometry.Alpha)
                end
                set(pct,'xdata',obj.Agents.Quad(1).Camera.Centroid{k}(:,1),...
                    'ydata',obj.Agents.Quad(1).Camera.Centroid{k}(:,2),...
                    'zdata',11+0*obj.Agents.Quad(1).Camera.Centroid{k}(:,3))
                if isempty(obj.Agents.Quad(1).Camera.DropSite{k})
                    set(pcds,'xdata',[],'ydata',[],'zdata',[])
                else
                    set(pcds,'xdata',obj.Agents.Quad(1).Camera.DropSite{k}(1,:),...
                        'ydata',obj.Agents.Quad(1).Camera.DropSite{k}(2,:),...
                        'zdata',obj.Agents.Quad(1).Camera.DropSite{k}(3,:))
                end
                
                pause(0.01)
                drawnow
                
                if obj.SaveVideo
                    frame = getframe(f);
                    writeVideo(mov,frame);
                end
                
            end
            
            if obj.SaveVideo
                close(mov)
            end
            
        end
        
        % PLOT STATES -----------------------------------------------------
        function PlotStates(obj)
            
            % Initialise options
            options = {'Select property to compare:'
                    ''
                    'Individual rigid-body states:'
                    'x, y, z, phi, theta, psi,'
                    'xdot, ydot, zdot, p, q, r,'
                    ''
                    'Grabber arm position:'
                    'xG, yG, zG, xGdot, yGdot, zGdot'
                    ''
                    'Combined rigid-body states:'
                    'position, attitude, velocity, ang rates,'
                    'Gposition, Gvelocity'
                    ''
                    'Camera gimbal states:'
                    'phiG, thetaG'
                    ''
                    'Other properties:'
                    'battery, airspeed'};
            Ind = [];
            
            fprintf('\n+- AGENT STATE COMPARISON -----------------\n|\n')
            while isempty(Ind)
                for i = 1:length(options)
                    fprintf('|  %s  \n',options{i})
                end
                fprintf('|\n')
                viz = input('|  Selection: ','s');

                % State selection
                fprintf('|\n|  Comparing results for %s...\n|\n',viz)

                % State indices
                StateArray = {'x','\itx \rm(m)'
                              'y','\ity \rm(m)'
                              'z','\itz \rm(m)'
                              'phi','\phi (rad)'
                              'theta','\theta (rad)'
                              'psi','\psi (rad)'
                              'xdot','\itxdot \rm(m/s)'
                              'ydot','\itydot \rm(m/s)'
                              'zdot','\itzdot \rm(m/s)'
                              'p','\itp \rm(rad/s)'
                              'q','\itq \rm(rad/s)'
                              'r','\itr \rm(rad/s)'
                              'phiG','\phi_G (rad)'
                              'thetaG','\theta_G (rad)'
                              'battery','Battery (V)'
                              'xG','\itxG \rm(m)'
                              'yG','\ityG \rm(m)'
                              'zG','\itzG \rm(m)'
                              'airspeed','\itV \rm(m/s)',
                              '0','-'};
                CombinedStates = {'position','Position (m)'
                              'attitude','Attitude (rad)'
                              'velocity','Velocity (m/s)'
                              'ang rates','Angular velocity (rad/s)'
                              'Gposition','Grabber position (m)'};
                if any(ismember(StateArray(:,1),viz))
                    Ind = find(ismember(StateArray(:,1),viz));
                    Type = 'Individual';
                    if strcmp(StateArray{Ind,1},'0')
                        fprintf('+------------------------------------------\n\n')
                        fprintf('Returning to main menu...\n\n')
                        return
                    end
                elseif any(ismember(CombinedStates(:,1),viz))
                    Ind = find(ismember(CombinedStates(:,1),viz));
                    Type = 'Combined';
                else
                    isempty(Ind)
                    fprintf('|  INVALID SELECTION\n\n')
                end
            
                fprintf('+------------------------------------------\n\n')
                fprintf('Returning to main menu...\n\n')
            end
            
            % Initialise figure
            f = figure('Name',['Comparison of state ',viz],...
                'Position',obj.CentreFig(obj.DimSmall));
            xlabel('Time (s)')
            if strcmp(Type,'Individual')
                ylabel(StateArray(Ind,2),'Interpreter','tex');
            else
                ylabel(CombinedStates(Ind,2),'Interpreter','tex');
            end
            box on, grid on, hold on
            xlim([obj.Time(1) obj.Time(end)])
            
            % Select states
            if strcmp(Type,'Combined')
                switch CombinedStates{Ind,1}
                    case {'position','attitude','velocity','ang rates'}
                        Ind = (1:3) + (Ind-1)*3;
                    case 'GPosition'
                        Ind = 16:18;
                end
            end
            
            % Loop agent types
            for j = 1:obj.NoAgentTypes
                
                Type = obj.AgentTypes{j};
                
                if strcmp(Type,'Quad')
                    
                    % Loop agents of type
                    for i = 1:obj.AgentTypeNos(j)
                        
                        plot(obj.Time,obj.Agents.(Type)(i).States(Ind,:));
                        if all(Ind <= 12)
                            plot(obj.Time,obj.Agents.(Type)(i).Commands(Ind,:),'--');
                        end
                        
                    end
                    
                end
                
            end
            
        end
        
        % PLOT INPUTS -----------------------------------------------------
        function PlotInputs(obj)
            
            % Initialise options
            options = {'Select property to compare:'
                    ''
                    'Individual inputs:'
                    'u1, u2, u3, u4, ucol, ulat, ulong, uyaw'
                    ''
                    'Combined inputs:'
                    'true, pseudo'};
            Ind = [];
            
            fprintf('\n+- AGENT INPUTS COMPARISON -------------------------\n\n')
            while isempty(Ind)
                for i = 1:length(options)
                    fprintf('|  %s  \n',options{i})
                end
                fprintf('|\n')
                viz = input('|  Selection: ','s');

                % Input selection
                fprintf('|\n|  Comparing results for %s...\n\n',viz)

                % Get state index
                InputArray = {'u1','u2','u3','u4','ucol','ulat','ulong','uyaw','true','pseudo'
                              '\itu\rm_1','\itu\rm_2','\itu\rm_3','\itu\rm_4',...
                              '\itu\rm_{col}','\itu\rm_{lat}','\itu\rm_{long}','\itu\rm_{yaw}',...
                              'Inputs','Pseudo-inputs'};
                Ind = find(ismember(InputArray(1,:),viz));
                if isempty(Ind)
                    fprintf('|  INVALID SELECTION\n\n')
                elseif strcmp(InputArray{1,Ind},'0')
                    fprintf('+------------------------------------------\n\n')
                    fprintf('Returning to main menu...\n\n')
                    return
                end
                
                fprintf('+------------------------------------------\n\n')
                fprintf('Returning to main menu...\n\n')
            end
            
            % Initialise figure
            f = figure('Name',['Comparison of input ',viz],...
                'Position',obj.CentreFig(obj.DimSmall));
            xlabel('Time (s)')
            ylabel(InputArray(2,Ind),'Interpreter','tex');
            box on, grid on, hold on
            
            % Select inputs
            if Ind == 9
                Ind = 1:4;
                Opt = 'Inputs';
            elseif Ind == 10
                Ind = 1:4;
                Opt = 'PseudoInputs';
            elseif Ind >= 5
                Ind = Ind - 4;
                Opt = 'PseudoInputs';
            else
                Opt = 'Inputs';
            end
            
            % Loop agent types
            for j = 1:obj.NoAgentTypes
                
                Type = obj.AgentTypes{j};
                
                % Only plot inputs if Type is Quad
                if strcmp(Type,'Quad')
                    
                    % Loop agents of type
                    for i = 1:obj.AgentTypeNos(j)
                        plot(obj.Time,obj.Agents.(Type)(i).(Opt)(Ind,:));
                    end
                end
                
            end
            
        end
        
    end
    
    % STATIC METHODS ------------------------------------------------------
    methods(Static)
        
        % CENTRE FIGURE IN WINDOW WITH SPECIFIC DIMENSIONS ----------------
        function pos = CentreFig(varargin)
            %CENTREFIG Provides values for figure('Position',[x,y,w,h]) which centre
            %the figure in the display.
            %
            %   [x,y,w,h] = CentreFig provides the position values for the default
            %   width, w = 560, and height, h = 420.
            %
            %   [x,y,w,h] = CentreFig(w) provides the position values for a square
            %   figure with side w.
            %
            %   [x,y,w,h] = CentreFig(w,h) provides the position values for a figure
            %   with width w and height h.

            switch nargin
                case 0
                    w = 560;    h = 420;
                case 1
                    if length(varargin{1}) == 1
                        w = varargin{1};
                        h = varargin{1};
                    elseif length(varargin{1}) == 2
                        w = varargin{1}(1);
                        h = varargin{1}(2);
                    else
                        error('Number of vector elements arguments cannot be greater than 2.')
                    end
                otherwise
                    error('Number of input arguments cannot be greater than 1.')
            end

            scrsz = get(0,'Screensize');
            S = scrsz(3:4);

            % Make sure figure fits in window
            D = [w h];
            D = S.*(D > S) + D.*(D <= S);
            w = D(1);   h = D(2);

            x = (S(1)-w)/2;
            y = (S(2)-h)/2;

            pos = [x y w h];

        end
        
    end
    
end

