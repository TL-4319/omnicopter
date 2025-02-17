function [posMat t] = createEllipse(inc,startPos,dt,t,a,e)
x0 = startPos(1);
y0 = startPos(2);
z0 = startPos(3);

RAAN = 0;

theta = linspace(-pi/2,3*pi/2,length(t))';
thetadot = 2*pi/t(end)';


r = sqrt(a^2*(1-e^2)./(1-e^2*cos(theta).^2));%a*(1-e^2)./(1+e*cos(theta));
%r = (a*(1-e^2))./(1+e*cos(theta));

%Position
x = r.*(cos(RAAN)*cos(theta)-sin(RAAN)*sin(theta)*cos(inc))+x0;%+(x0-r);
y = r.*(sin(RAAN)*cos(theta)+cos(RAAN)*sin(theta)*cos(inc))+y0;%+y0;
z = r.*sin(inc).*sin(theta)+z0;

%% Ramp In/Ramp Out
xss = startPos(1)*ones(length(t),1);
yss = startPos(2)*ones(length(t),1);
zss = startPos(3)*ones(length(t),1);

val = 50;
trans = 10; %transit time in sec
t1 = t(1:val:end-1)-t(end);
t2 = t(1:val:end)+trans;
t3 = t(1:val:end)+t2(end)+trans;
ttot = [t1 t2 t3];
ttraj = t+trans;
tmeas = [0:dt:trans ttraj(2:end-1) ttraj(end):dt:(trans+ttraj(end))];
x = spline(ttot,[xss(1:val:end-1); x(1:val:end); xss(1:val:end-1)],tmeas)';
y = spline(ttot,[yss(1:val:end-1); y(1:val:end); yss(1:val:end-1)],tmeas)';
z = spline(ttot,[zss(1:val:end-1); z(1:val:end); zss(1:val:end-1)],tmeas)';
t = tmeas;


%%
% %Velocity
% vx = -r.*cos(inc).*thetadot.*sin(theta);
% vy = r.*cos(inc).*thetadot.*cos(theta);
% vz = sin(inc)*thetadot.*cos(theta);
%
% %Acceleration
% ax = -r.*cos(inc)*thetadot^2.*cos(theta);
% ay = -r.*cos(inc)*thetadot^2.*sin(theta);
% az = sin(inc)*thetadot^2.*(-sin(theta));

vx = [0 ;(x(2:end)-x(1:end-1))/dt];
vy = [0; (y(2:end)-y(1:end-1))/dt];
vz = [0; (z(2:end)-z(1:end-1))/dt];

ax = [0; (vx(2:end)-vx(1:end-1))/dt];
ay = [0; (vy(2:end)-vy(1:end-1))/dt];
az = [0; (vz(2:end)-vz(1:end-1))/dt];

posMat = [x,y,z,vx,vy,vz,ax,ay,az];
end