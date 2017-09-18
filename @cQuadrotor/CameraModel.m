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