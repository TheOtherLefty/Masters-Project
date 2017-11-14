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
% if rem(obj.Time,obj.dt_Controller) < 1e-6
    if ~strcmp(obj.Mode,'Testing')
        
        obj.Mode = obj.StateMachine(obj.Mode);
        
    else
        
        Commands = [0 0 -0.5 0]';
        
        % State reconstruction
        obj.StatesEst = obj.StateReconstruction(obj.Outputs);
        
        % Update controller
        [obj.Inputs,obj.PseudoInputs,obj.Commands] = ...
            obj.Controller(obj.StatesEst,Commands,obj.Time,0);
        
    end
% end

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
    'Outputs','Commands','Mode','Status','BatteryLevel','TargetCount'};
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