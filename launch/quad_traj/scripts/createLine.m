function [posMat t] = createLine(style,startPos,limPos,t,dt)
chunk = floor(length(t)/4);
halt = dt:dt:2;

switch style
    case 'hover'
        x = startPos(1)*ones(length(t),1);
        y = startPos(2)*ones(length(t),1);
        z = startPos(3)*ones(length(t),1);
        t=t;
    case 'x'
        x1 = linspace(startPos(1),limPos(1,2),chunk+1);
        x2 = linspace(limPos(1,2),limPos(1,1),2*chunk+1);
        x3 = linspace(limPos(1,1),startPos(1),chunk+1);
        x = [x1 x1(end)*ones(1,length(halt)) x2(1:end-1) x2(end-1)*ones(1,length(halt)) x3(1:end-1)]';
        y = startPos(2)*ones(1,length(t)+2*length(halt))';
        z = startPos(3)*ones(1,length(t)+2*length(halt))';
        t = t(1):dt:(t(end)+halt(end)*2);
        
    case 'y'
        y1 = linspace(startPos(2),limPos(2,2),chunk+1);
        y2 = linspace(limPos(2,2),limPos(2,1),2*chunk+1);
        y3 = linspace(limPos(2,1),startPos(2),chunk+1);
        y = [y1 y1(end)*ones(1,length(halt)) y2(1:end-1) y2(end-1)*ones(1,length(halt)) y3(1:end-1)]';
        x = startPos(1)*ones(1,length(t)+2*length(halt))';
        z = startPos(3)*ones(1,length(t)+2*length(halt))';
        t = t(1):dt:(t(end)+halt(end)*2);
    case 'z'
        z1 = linspace(startPos(3),limPos(3,2),chunk+1);
        z2 = linspace(limPos(3,2),limPos(3,1),2*chunk+1);
        z3 = linspace(limPos(3,1),startPos(3),chunk+1);
        z = [z1 z2(1:end-1) z3(1:end-1)]';
        x = startPos(1)*ones(1,length(t))';
        y = startPos(2)*ones(1,length(t))'; 
        
end

x0 = startPos(1)*ones(length(t),1);
y0 = startPos(2)*ones(length(t),1);
z0 = startPos(3)*ones(length(t),1);

val = 50;
x = spline([t(1:val:end-1) t(1:val:end)+t(end)],[x0(1:val:end-1); x(1:val:end)],t+t(end))';
y = spline([t(1:val:end-1) t(1:val:end)+t(end)],[y0(1:val:end-1); y(1:val:end)],t+t(end))';
z = spline([t(1:val:end-1) t(1:val:end)+t(end)],[z0(1:val:end-1); z(1:val:end)],t+t(end))';

vx = [0 ;(x(2:end)-x(1:end-1))/dt];
vy = [0; (y(2:end)-y(1:end-1))/dt];
vz = [0; (z(2:end)-z(1:end-1))/dt];

ax = [0; (vx(2:end)-vx(1:end-1))/dt];
ay = [0; (vy(2:end)-vy(1:end-1))/dt];
az = [0; (vz(2:end)-vz(1:end-1))/dt];

posMat = [x,y,z,vx,vy,vz,ax,ay,az];

end