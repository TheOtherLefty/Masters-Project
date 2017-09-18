function [Cvisd,TargetFound,TargetVis] = ObjectTracking(obj,Vc,Vdsc)

% Get scene geometry info
F = obj.Camera.Faces;
C = obj.Camera.Colours;
res = obj.Camera.Res;

Cup = obj.CurrentColours + obj.Cvar;
Clw = obj.CurrentColours - obj.Cvar;
numC = size(obj.CurrentColours,1);

% Preallocate arrays
Fid = cell(1,numC);
Fvis = cell(1,numC);
Cvis = cell(1,numC);

% Find indices of faces within search range
for c = 1:numC
    Fidc = zeros(size(C));
    for j = 1:3
        Fidc(:,j) = and(C(:,j) <= Cup(c,j),C(:,j) >= Clw(c,j));
    end
    Fid{c} = all(Fidc,2);
    
    % Eliminate faces outside of range
    Fvis{c} = F(Fid{c},:);
    Cvis{c} = C(Fid{c},:);
end

% Find vertices corresponding to these faces and get centroid
Vu = cell(1,numC);
Cntrd = zeros(size(obj.CurrentColours));
obj.Ccheck = zeros(size(obj.CurrentColours,1),1);
for c = 1:numC
    
    Fvisc = reshape(Fvis{c},[],1);
    Fvisc(isnan(Fvisc)) = [];
    
    % Find vertices
    Vu{c} = Vc(Fvisc,:);
    
    % Eliminate vertices outside of FOV
    Vout = [];
    for i = 1:size(Vu{c},1)
        if Vu{c}(i,1) > -res(1)/2 && Vu{c}(i,1) < res(1)/2 ...
                && Vu{c}(i,2) > -res(2)/2 && Vu{c}(i,2) < res(2)/2 ...
                && Vu{c}(i,3) > 0
            Vout = [Vout
                Vu{c}(i,:)];
        end
    end
    Vu{c} = Vout;
    
    if ~isempty(Vu{c})
        Cntrd(c,:) = sum(Vu{c},1)/size(Vu{c},1);
        obj.Ccheck(c,1) = 1;
    else
        Cntrd(c,:) = NaN*[1 1 1];
        obj.Ccheck(c,1) = 0;
    end
    
end

% Save centroids to camera property
obj.Camera.Centroid = Cntrd;

% Remove NaNs from array
Cvis = Cntrd(~all(isnan(Cntrd),2),:);

% Remove centroids in drop site
% Estimate drop radius, do this properly later
DS = sum(Vdsc,2)/size(Vdsc,2);
CR = obj.Camera.FocalLength*obj.DropRadius/abs((obj.States(3)-obj.CameraPosition(3)));
Cvisd = [];
D = [];
for c = 1:size(Cvis,1)
    D(c) = norm(Cvis(c,1:2)-DS(1:2)');
end
if ~isempty(D)
    Cvisd = Cvis(D>CR,:);
end

% If target is within field of view and outside drop zone, it
% is visible
if isempty(Cvisd)
    TargetVis = 0;
else
    TargetVis = 1;
end

% Remove centroids outside of boundary
Cvisb = [];
B = [];
for c = 1:size(Cvisd,1)
    B(c) = norm(Cvisd(c,1:2));
end
if ~isempty(B)
    Cvisb = Cvisd(B<obj.Camera.BndRadius,:);
end
%             Cvisb = Cvis(and(abs(Cvis(:,1))<obj.Camera.Bnd(1)/2,...
%                 abs(Cvis(:,2))<obj.Camera.Bnd(2)/2),:);

% Sort centroids by distance from camera centre
Cnorm = zeros(size(Cvisd,1),1);
for c = 1:size(Cvisd,1)
    Cnorm(c,1) = norm(Cvis(c,1:2));
end
if ~isempty(Cvisd)
    Cvisd = [Cvisd Cnorm];
    Cvisd = sortrows(Cvisd,4);
    Cvisd = Cvisd(:,1:3);
end

% If no centroids are left, target is not found
if isempty(Cvisb)
    TargetFound = 0;
else
    TargetFound = 1;
end

if obj.Time > 163 && obj.Time < 170 && 0
    fprintf('Time = %.2f s\n',obj.Time)
    fprintf('TargetFound = %.0f\n',TargetFound)
    fprintf('DropSite = [ ')
    fprintf('%.3f ',DS')
    fprintf(']\n')
    fprintf('Cvis = [ ')
    fprintf('%.3f ',Cvis')
    fprintf(']\n')
    fprintf('Cvisd = [ ')
    fprintf('%.3f ',Cvisd')
    fprintf(']\n')
    fprintf('Cvisb = [ ')
    fprintf('%.3f ',Cvisb')
    fprintf(']\n\n')
end

% Remove entry from array
%             obj.CurrentColours = obj.CurrentColours(~obj.Ccheck,:);