function InitWaypoints(obj)
    
% Grid size and step
gX = obj.GridSize(1);
gY = obj.GridSize(2);
dX = obj.CellSize;

% X range
wpx = (0:gX-1)*dX;

% Constant height
wpz = -1.5;

% Repeat and flip over y range
WP = [];
off = 0;
dir = 1;
for i = 1:gY

    WP = [WP
          off+dir*wpx' zeros(gX,1)+dX*(i-1) zeros(gX,1)+wpz zeros(gX,1)];
    off = off + max(wpx)*dir;
    dir = -dir;

end
   
% Order rows properly if decision-making is active
if ~isempty(obj.Decisions)
    
    WP = sortrows(WP, 1);
    WP = sortrows(WP, 2);
    
end

obj.Waypoints = WP;
obj.MaxWP = size(WP,1);