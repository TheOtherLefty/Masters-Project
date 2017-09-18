function Y = Sensors(obj,Y,X,Xdot,t)

% Continuous states
% X = [x y z phi theta psi xdot ydot zdot p q r]'

% Outputs
% Y = [ax ay az gx gy gz]'

% Optitrack
Y(1:6,1) = obj.Optitrack(X,t);

% IMU
Y(7:12,1) = obj.IMU(X,Xdot,t);