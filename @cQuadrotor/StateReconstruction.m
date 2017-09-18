function X = StateReconstruction(obj,Y)

% Position
X(1:3,1) = Y(1:3);

% Attitude
X(4:6,1) = Y(4:6);

% Velocity
X(7:9,1) = obj.VelFilter*(Y(1:3) - obj.VelInt);
obj.VelInt = obj.VelInt + obj.dt_Controller*X(7:9);

% Angular velocity
X(10:12,1) = Y(10:12);