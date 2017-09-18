function Faults(obj)

% Grabber fault, can occur at any time but only makes a
% difference when transporting objects
if rand > (1 - obj.P_GrabberFailure)^(obj.TimeStep/60)
    obj.GrabberActive = 0;
end

% Actuator fault, can occur at any time
if ~all(obj.ActuatorFaults) && rand > (1 - obj.P_ActuatorFailure)^(obj.TimeStep/60)
    ind = ceil(4*rand);
    obj.ActuatorFaults(ind) = 1;
end