function State = FaultDetection(obj, X, State)

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