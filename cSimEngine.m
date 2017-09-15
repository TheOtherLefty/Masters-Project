classdef cSimEngine < handle
    %cSIMENGINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % Time properties
        Time;
        TimeFinal;
        TimeStep;
        SampleStep;
        
        % Environment object
        Environment;
        
        % Agent list - structure containing agents in the format
        %   Agents.<AgentType>(<AgentNo>) = <AgentObject>
        Agents;
        
        % Agent types
        AgentTypes;
        NoAgentTypes;
        AgentTypeNos;
        
        % Blackboard - structure containing agent data in the format
        %   Blackboard.<AgentID>.<AgentProperty>
        Blackboard;
        
        % SaveData object - collects agent and time data at sample times
        % and saves them for output after the simulation has been run
        BlackBox;
        
        % Waitbar
        Progress;
        ProgressBar = 0;
        
    end
    
    methods
        
        % CLASS CONSTRUCTOR -----------------------------------------------
        function obj = cSimEngine(BlackBox,Environment,Agents,t,tfin,dt,tsamp)
            
            if obj.ProgressBar
                obj.Progress = waitbar(0,'Initialising simulation');
            end
            
            % Initialise time properties
            obj.Time = t;
            obj.TimeFinal = tfin;
            obj.TimeStep = dt;
            obj.SampleStep = tsamp;
            
            % Initialise environment
            obj.Environment = Environment;
            
            % Initialise agents
            obj.Agents = Agents;
            obj.AgentTypes = fieldnames(Agents);
            obj.NoAgentTypes = length(obj.AgentTypes);
            for i = 1:obj.NoAgentTypes
                obj.AgentTypeNos(i) = length(Agents.(obj.AgentTypes{i}));
            end
            
            % Provide target numbers to quadrotor
            for i = 1:length(obj.Agents.Quad)
                Agents.Quad(i).NumTargets = length(obj.Agents.Target);
            end
            
            % Initialise blackboard
            obj.InitialiseBlackboard;
            
            % Initialise geometry
            obj.InitialiseGeometry;
            
            % Initialise blackbox
            BlackBox.Initialise(obj.Blackboard);
            obj.BlackBox = BlackBox;
            
        end
        
        % INITIALISE BLACKBOARD -------------------------------------------
        function InitialiseBlackboard(obj)
            
            % Initialise time and environment
            obj.Blackboard.GlobalTime = obj.Time;
            obj.Blackboard.Environment = [];
            
            % Cycle agent types
            for j = 1:obj.NoAgentTypes
                
                % Current agent type
                Type = obj.AgentTypes{j};

                % No. of agents of current type
                NoAgents = length(obj.Agents.(Type));

                % Cycle agents of current type
                for i = 1:NoAgents
                    
                    ID = obj.Agents.(Type)(i).ID;
                    
                    % Initialise environment
%                     obj.Agents.(Type)(i).InitEnvironment(obj.Environment);
                    
                    % Initialise blackboard entry for agent
                    Props = {'Time','States','Inputs','PseudoInputs','Outputs',...
                        'Commands','Mode','Status','Geometry'};
                    for p = 1:length(Props)
                        obj.Blackboard.Agents.(Type)(i).(Props{p})...
                            = obj.Agents.(Type)(i).(Props{p});
                    end
                    
                    % Initialise unique properties for quadrotor
                    if strcmp(Type,'Quad')
%                         obj.Blackboard.Agents.(Type)(i).Camera.Vertices = [];
                        obj.Blackboard.Agents.(Type)(i).Camera = obj.Agents.(Type)(i).Camera;
                    end
                    
                    % Set final time of agent to final time of sim
                    obj.Agents.(Type)(i).TimeFinal = obj.TimeFinal;

                    % Override chosen global time-step if too large
                    if obj.TimeStep > obj.Agents.(Type)(i).TimeStep
                        obj.TimeStep = obj.Agents.(Type)(i).TimeStep;
                    end
                    
                end
                
            end
            
            % Ensure sample time is multiple of step size
            obj.SampleStep = ceil(obj.SampleStep/obj.TimeStep)*obj.TimeStep;
            
        end
        
        % INITIALISE GEOMETRY ---------------------------------------------
        function InitialiseGeometry(obj)
            
            % Initialise combined geometry for environment
            Faces = obj.Environment.Geometry.Faces;
            Vertices = obj.Environment.Geometry.Vertices;
            Colours = obj.Environment.Geometry.Colours;
            Alpha = obj.Environment.Geometry.Alpha;
            DropSite = obj.Environment.Geometry.DropSite;
            DropLocation = obj.Environment.Geometry.DropLocation;
            DropRadius = obj.Environment.Geometry.DropRadius;
            
            % Set animation limits from environment geometry
            Limits = [min(Vertices)' max(Vertices)'];
            
            % Cycle agents and retrieve geometry
            for j = 1:obj.NoAgentTypes
                
                Type = obj.AgentTypes{j};
                
                for i = 1:obj.AgentTypeNos(j)
                    
                    % Current Agent
                    Agent = obj.Agents.(Type)(i);
                    
                    % Updates Faces
                    F = Agent.Geometry.Faces + size(Vertices,1);
                    if size(F,2) == size(Faces,2)
                        Faces = [Faces; F];
                    elseif size(F,2) > size(Faces,2)
                        Faces = [Faces NaN*ones(size(Faces,1),size(F,2)-size(Faces,2))
                                 F];
                    else
                        Faces = [Faces
                                 F  NaN*ones(size(F,1),size(Faces,2)-size(F,2))];
                    end
                    
                    % Vertex position identifier
                    Agent.Geometry.ID = size(Vertices,1)+1;
                    
                    % Update vertices
                    Vertices = [Vertices; Agent.Geometry.Vertices];
                    
                    % Update colours
                    Colours = [Colours; Agent.Geometry.Colours];
                    
                    % Update alpha
                    Alpha = [Alpha; Agent.Geometry.Alpha];
                    
                end
                
            end
            
            % Save to blackboard
            obj.Blackboard.Geometry.Faces = Faces;
            obj.Blackboard.Geometry.Vertices = Vertices;
            obj.Blackboard.Geometry.CameraVertices = NaN;
            obj.Blackboard.Geometry.Colours = Colours;
            obj.Blackboard.Geometry.Alpha = Alpha;
            obj.Blackboard.Geometry.Limits = Limits;
            obj.Blackboard.Geometry.DropSite = DropSite;
            obj.Blackboard.Geometry.DropLocation = DropLocation;
            obj.Blackboard.Geometry.DropRadius = DropRadius;
            
        end
        
        % SIMULATION LOOP -------------------------------------------------
        function BlackBox = SimLoop(obj)
            
            if obj.ProgressBar
                waitbar(0,obj.Progress,'Running simulation')
            end
            
            MissionEnd = 0;
            
            while obj.Time < obj.TimeFinal && ~MissionEnd
                
                % Loop agent types
                for j = 1:obj.NoAgentTypes
                    
                    Type = obj.AgentTypes{j};
                    
                    % Loop agents
                    for i = 1:obj.AgentTypeNos(j)
                        
                        % Current agent
                        Agent = obj.Agents.(Type)(i);
                        
                        if obj.Time > obj.Blackboard.Agents.(Type)(i).Time
                            
                            % Update blackboard
                            obj.Blackboard = Agent.UpdateAgent(obj.Blackboard,i);
                            
                        end
                        
                        if strcmp(Type,'Quad')
                            MissionEnd = Agent.MissionEnd;
                        end
                        
                    end
                    
                end
                
                % Update global time
                obj.Time = round(obj.Time/obj.TimeStep + 1)*obj.TimeStep;
                obj.Blackboard.GlobalTime = obj.Time;
                
                if obj.ProgressBar
                    waitbar(obj.Time/obj.TimeFinal,obj.Progress,'Running simulation')
                end
                
                % Save data to black box on sample step
                if rem(obj.Time,obj.SampleStep) < obj.TimeStep/10
                    obj.BlackBox.Update(obj.Blackboard)
                end
                
            end
            
            % Close waitbar
            if obj.ProgressBar
                close(obj.Progress)
            end
            
            % Export blackbox data to workspace
            BlackBox = obj.BlackBox;
            
        end
        
    end
    
end

