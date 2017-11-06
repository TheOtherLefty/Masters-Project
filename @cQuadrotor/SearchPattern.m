function [Yd,Exitflag] = SearchPattern(obj,Y,t)

% Track waypoints
if norm(Y([1:3,6])-obj.Waypoints(obj.WP,1:4)') < 1e-2
    obj.WP = obj.WP+1;
end

if obj.WP > obj.MaxWP
    obj.WP = obj.MaxWP;
    Exitflag = 1;
else
    Exitflag = 0;
end

Yd = obj.Waypoints(obj.WP,:)';