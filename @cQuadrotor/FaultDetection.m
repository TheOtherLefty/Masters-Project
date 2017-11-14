function State = FaultDetection(obj,X,State)

% Battery monitor
% if X(15) < obj.WarningLevel && ~obj.BatteryWarning
if obj.BatteryLevel <= 0 && ~obj.BatteryWarning
    fprintf('Time %4.2f s: BATTERY WARNING\n',obj.Time)
    obj.Cint = 0;
    State = 'Return to base';
    obj.BatteryWarning = 1;
end

% Grabber fault
if ~obj.GrabberActive && ismember(State,{'Ascend','Transport'})
    fprintf('Time %4.2f s: TARGET DROPPED\n',obj.Time)
    State = 'Reacquire target';
    obj.GrabberWarning = 1;
end

% Actuator fault monitor
if any(obj.ActuatorFaults) && ~obj.ActuatorFaultWarning
    fprintf('Time %4.2f s: FAULTY ACTUATOR\n',obj.Time)
    State = 'Emergency land';
    obj.ActuatorFaultWarning = 1;
end