function Xdot = Dynamics(obj,X,U,t)

% Continuous states
% X = [x y z phi theta psi xdot ydot zdot p q r]'

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

J = [1  0         -sin(X(5))
    0  cos(X(4))  sin(X(4))*cos(X(5))
    0 -sin(X(4))  cos(X(4))*cos(X(5))];

% Update mass and inertias if object attached
if obj.GrabberActive
    m = obj.mQ + obj.mO;
else
    m = obj.mQ;
end

% Inertias
I = diag([obj.Ix obj.Iy obj.Iz]);

% ROTOR MODEL

% Fault model
U = ~obj.ActuatorFaults.*U;

% Limit inputs
if obj.ActuatorLimits
    U(U<0) = 0;
    U(U>0.05) = 0.05;
end

% Individual thrust and torque
Ti = obj.Kt*U;
Qi = obj.Kq*U.*[-1 -1 1 1]';

% Net thrust and torque
T = sum(Ti);
Q = sum(Qi);

% Rotor positions around CG
Ri = [-obj.L obj.L 0 0
    0 0 -obj.L obj.L
    0 0 0 0];

% GROUND MODEL

% Floor spring and damping
kf = m*obj.FloorNatFreq^2;
cf = m*2*obj.FloorDamp*obj.FloorNatFreq;

if X(3) > -obj.EffRadius
    G = Reb*[-cf*X(7:8)
        kf*(-obj.EffRadius-X(3)) - cf*X(9)];
    Gm = -0.01*[kf*X(4:5) + cf*X(10:11)
        cf*X(12)];
else
    G = [0 0 0]';
    Gm = [0 0 0]';
end

% FORCES AND MOMENTS

% Forces
F = [0 0 -T]' + G;

% Moments
M = [0 0 0]' + Gm;
for i = 1:length(U)
    M = M + cross(Ri(:,i),[0 0 -Ti(i)]') + [0 0 Qi(i)]';
end

% BATTERY MODEL
if X(15) >= obj.MaxBattery && obj.BatteryRate > 0
    Vrate = 0;
elseif X(15) <= 0 && obj.BatteryRate < 0
    Vrate = 0;
else
    Vrate = obj.BatteryRate;
end

% Grabber dynamics
rB = [0 0 obj.ArmLength]';

% STATE DERIVATIVES
Xdot = [X(7:9)
    J\X(10:12)
    Rbe*F/m + [0 0 obj.g]';
    I\M
    (-X(4:5)-X(13:14))/obj.Gtau
    Vrate
    X(19:21)
    Rbe*F/m + [0 0 obj.g]' + Rbe*(cross(X(10:12),cross(X(10:12),rB)) + cross(I\M,rB))];

% Checks
%             if obj.Time > 140 && strcmp(obj.Mode,'Search')
%                 fprintf('Time = %.2f s\n',t)
%                 fprintf('Commands = ['),fprintf('%.3f ',obj.Commands([1:3, 6])),fprintf(']\n')
%                 fprintf('Inputs = [ '),fprintf('%.3f ',U),fprintf(']\n')
%                 fprintf('Forces = [ '),fprintf('%.3f ',F),fprintf(']\n')
%                 fprintf('Floor = [ '),fprintf('%.3f ',G),fprintf(']\n')
%                 fprintf('Moments = [ '),fprintf('%.3f ',M),fprintf(']\n')
%                 fprintf('Floor = [ '),fprintf('%.3f ',Gm),fprintf(']\n')
%                 fprintf('Pos = [ '),fprintf('%.3f ',X(1:3)),fprintf(']\n')
%                 fprintf('Vel = [ '),fprintf('%.3f ',X(4:6)),fprintf(']\n')
%                 fprintf('Acc = [ '),fprintf('%.3f ',Xdot(4:6)),fprintf(']\n\n')
%                 pause
%             end