function InitWaypoints(obj)

% Define waypoints
dR = 0.5;

Waypoints = [obj.RoomLimits(1,1)+dR obj.RoomLimits(2,1)+dR -2 0*pi/2];
i = 1;

while (max(Waypoints(:,1)) < obj.RoomLimits(1,2)-dR) || (Waypoints(i,2) ~= obj.RoomLimits(2,2)-dR)
    
    i = i + 1;
    
    Waypoints(i,:) = Waypoints(i-1,:);
    
    if rem(i,2) > 1e-3
        Waypoints(i,1) = Waypoints(i-1,1) + dR;
        if rem(i+1,4) > 1e-3
            Waypoints(i,4) = 0*pi/2;
        else
            Waypoints(i,4) = -0*pi/2;
        end
    else
        if rem(i,4) > 1e-3
            Waypoints(i,2) = obj.RoomLimits(2,2)-dR;
        else
            Waypoints(i,2) = obj.RoomLimits(2,1)+dR;
        end
    end
    
end

obj.Waypoints = Waypoints;
obj.MaxWP = i;