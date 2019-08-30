function InitWaypoints(obj)
    
% Grid size and step
gX = obj.GridSize(1);
gY = obj.GridSize(2);
dX = obj.CellSize;

% X range
wpx = (0:gX-1)*dX;

% Constant height
wpz = -1;

% Repeat and flip over y range
WP = [];
off = -(gY/2)*dX;
dir = 1;
for i = off:dX:-off-dX

    WP = [WP
          off+dir*wpx' zeros(gX,1)+i zeros(gX,1)+wpz zeros(gX,1)];
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