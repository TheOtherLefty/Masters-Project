function InitialiseControllers(obj)

% CLOSED-LOOP RESPONSE CHARACTERISTICS

% Height
tsz = 2;
zetaz = 1;
wnz = 3.9/(tsz*zetaz);
az = 0*wnz;

obj.Kz(3) = 2*zetaz*wnz + az;
obj.Kz(2) = (2*az*zetaz*wnz + wnz^2)/obj.Kz(3);
obj.Kz(1) = az*wnz^2/obj.Kz(3);

% Position
tsp = 2;
zetap = 1;
wnp = 3.9/(tsp*zetap);
ap = 0*wnp;

obj.Kx(3) = 2*zetap*wnp + ap;
obj.Kx(2) = (2*ap*zetap*wnp + wnp^2)/obj.Kx(3);
obj.Kx(1) = ap*wnp^2/obj.Kx(3);
obj.Ky = obj.Kx;

% Roll and pitch
zetaa = 1;
wna = 10*wnp;
tsa = 3.9/(zetaa*wna);

obj.Kphi(1) = wna^2;
obj.Kphi(2) = 2*zetaa*wna;
obj.Ktheta = obj.Kphi;

% Yaw
tsy = 4;
zetay = 1;
wny = 3.9/(zetay*tsy);

obj.Kpsi(1) = wny^2;
obj.Kpsi(2) = 2*zetay*wny;