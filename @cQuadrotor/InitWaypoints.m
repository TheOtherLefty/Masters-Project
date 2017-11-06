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

obj.Waypoints = WP;
obj.MaxWP = size(WP,1);