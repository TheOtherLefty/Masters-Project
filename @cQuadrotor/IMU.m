function Y = IMU(obj,X,Xdot,t)

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
%             Rbe = Reb';

% Accelerometer
Y(1:3,1) = Reb*(Xdot(7:9) - [0 0 obj.g]');

% Gyroscope
Y(4:6,1) = X(10:12);

% Magnetometer