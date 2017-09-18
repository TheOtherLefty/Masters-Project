function Status = StatusMonitor(obj)

if obj.ActuatorFaultWarning
    Status = 'Actuator fault';
elseif obj.GrabberWarning
    Status = 'Grabber fault';
elseif obj.BatteryWarning
    Status = 'Low battery';
elseif obj.SystemsWarning
    Status = 'System fault';
else
    Status = 'Fine';
end