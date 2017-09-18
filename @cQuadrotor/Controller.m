function [U,Up,Yd] = Controller(obj,X,Yd,t,Vswitch)

% Reconstruct states
Pos = X(1:3);
Att = X(4:6);
Vel = X(7:9);
AngVel = X(10:12);

% Position commands
if Vswitch
    PosDes = [NaN NaN Yd(3)]';
else
    PosDes = Yd(1:3);
end

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
if any(isnan(obj.epos_int))
    obj.epos_int = [0 0 0]';
else
    obj.epos_int = obj.epos_int + obj.dt_Controller*epos;
end

if Vswitch
    VelDes = [Yd(1:2)
              Kp(3)*epos(3) + Ki(3)*obj.epos_int(3)];
else
    VelDes = Kp.*epos + Ki.*obj.epos_int;
end


% Limit velocity
VelDes = VelDes.*(abs(VelDes)<=obj.VelLimit) + obj.VelLimit*(VelDes>obj.VelLimit)...
    - obj.VelLimit*(VelDes<-obj.VelLimit);

% Velocity controller
Acc = Kd.*(VelDes - Vel);

% Limit acceleration
Acc = Acc.*(abs(Acc)<=obj.AccLimit) + obj.AccLimit*(Acc>obj.AccLimit)...
    - obj.AccLimit*(Acc<-obj.AccLimit);

uz = m*(obj.g - Acc(3))/(obj.Kt*cos(Att(1))*cos(Att(2)));
% AttDes(1) = -(vx*sin(Att(3)) - vy*cos(Att(3)))/obj.g;
% AttDes(2) = -(vx*cos(Att(3)) + vy*sin(Att(3)))/obj.g;

AttDes(1) = asin(m*(Acc(2)*cos(Att(3)) - Acc(1)*sin(Att(3)))/(obj.Kt*uz));
AttDes(2) = -asin(m*(Acc(1)*cos(Att(3)) + Acc(2)*sin(Att(3)))/(obj.Kt*uz*cos(Att(1))));

for i = 1:2
    if abs(AttDes(i)) > obj.AttLimit
        AttDes(i) = sign(AttDes(i))*obj.AttLimit;
    end
end

% Attitude control
vphi = obj.Kphi*[AttDes(1) - Att(1);
    AngVelDes(1) - AngVel(1)];
vtheta = obj.Ktheta*[AttDes(2) - Att(2);
    AngVelDes(2) - AngVel(2)];
vpsi = obj.Kpsi*[AttDes(3) - Att(3);
    AngVelDes(3) - AngVel(3)];

uphi = obj.Ix*vphi/(obj.Kt*obj.L);
utheta = obj.Iy*vtheta/(obj.Kt*obj.L);
upsi = obj.Iz*vpsi/obj.Kq;

Up = [uz uphi utheta upsi]';

% Get true rotor inputs from pseudo-controls
C = [ 1  1 1  1
    0  0 1 -1
    -1  1 0  0
    -1 -1 1  1];
Cinv = C'/(C*C');

U = Cinv*Up;

% Limit inputs
if obj.ActuatorLimits
    U(U<0) = 0;
    U(U>0.05) = 0.05;
end

% if any(isnan(U))
%     Vswitch
%     PosDes
%     VelDes
%     AttDes
%     U
%     pause
% end

% Output commands
Yd = [PosDes; AttDes; VelDes; AngVelDes];