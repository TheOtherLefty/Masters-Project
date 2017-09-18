function [Vxy,C] = VisionController(obj,Coords,X)

% Align camera axes with body
C = [Coords(1,2) Coords(1,1)]';

% Scale error
Cs = C*abs((X(3)-obj.CameraPosition(3)))/obj.Camera.FocalLength;

% Gains
Kp = 0.35*obj.Kx(2);
Ki = 0.0001;

% Integral error
obj.Cint = obj.Cint + obj.dt_Controller*Cs;

% Center coords in frame of reference
Vxy = Kp*Cs + Ki*obj.Cint;

% Fix frame of reference
Rpsi = [cos(X(6)) -sin(X(6))
        sin(X(6))  cos(X(6))];
Vxy = Rpsi*Vxy;