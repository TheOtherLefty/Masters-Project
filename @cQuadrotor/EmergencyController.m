function [U,Up,Yd] = EmergencyController(obj,X,Yd,t,Faults)

% Reconstruct states
Pos = X(1:3);
Att = X(4:6);
Vel = X(7:9);
AngVel = X(10:12);

% Position commands
PosDes = [NaN NaN Yd(3)]';

% Set mass
if obj.GrabberActive
    m = obj.mQ + obj.mO;
else
    m = obj.mQ;
end

% Known attitude commands
AttDes = [0 0 wrapToPi(Yd(4))]';
AngVelDes = [0 0 0]';

% Position controller gains
Ki = [obj.Kx(1) obj.Ky(1) obj.Kz(1)]';
Kp = [obj.Kx(2) obj.Ky(2) obj.Kz(2)]';
Kd = [obj.Kx(3) obj.Ky(3) obj.Kz(3)]';

% Errors
epos = PosDes - Pos;
obj.epos_int = obj.epos_int + obj.dt_Controller*epos;

% Velocity command
VelDes = [Yd(1:2)
          Kp(3)*epos(3) + Ki(3)*obj.epos_int(3)];

% Limit velocity
VelDes = VelDes.*(abs(VelDes)<=obj.VelLimit) + obj.VelLimit*(VelDes>obj.VelLimit)...
    - obj.VelLimit*(VelDes<-obj.VelLimit);

% Velocity controller
Acc = Kd.*(VelDes - Vel);

% Limit acceleration
Acc = Acc.*(abs(Acc)<=obj.AccLimit) + obj.AccLimit*(Acc>obj.AccLimit)...
    - obj.AccLimit*(Acc<-obj.AccLimit);

uz = obj.mQ*(obj.g - Acc(3))/(obj.Kt*cos(Att(1))*cos(Att(2)));
%             uz = 2*obj.m*(obj.g - Acc(3))/(obj.Kt*cos(Att(1))*cos(Att(2)));

Up = [uz 0 0 0]';

% Get true rotor inputs from pseudo-controls
U = uz*0.5*[1 1 1 1]';
if any(find(Faults) == [1,2])
    U(1:2) = [0 0];
elseif any(find(Faults) == [3,4])
    U(3:4) = [0 0];
else
    error('This shouldn''t happen')
end

% Limit inputs
U(U<0) = 0;
U(U>0.05) = 0.05;

% Output commands
Yd = [PosDes; AttDes; VelDes; AngVelDes];