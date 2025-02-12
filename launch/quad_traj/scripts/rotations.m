function [attMat,t] = rotations(style,deg,t,dt,axis)
halt = dt:dt:10;
switch style
    case 'roll'
        yaw = zeros(length(t)+2*length(halt),1);
        pitch = zeros(length(t)+2*length(halt),1);
        if deg < 360
            r1 = linspace(0,deg*pi/180,length(t)/4+1);
            r2 = linspace(deg*pi/180,-deg*pi/180,length(t)/4*2+1);
            r3 = linspace(-deg*pi/180,0,length(t)/4+1);
            roll = [r1 r1(end)*ones(1,length(halt)) r2(1:end-1)...
                r2(end-1)*ones(1,length(halt))  r3(2:end)]';
        else
            r1 = linspace(0,2*pi,length(t)/2+1);
            r2 = linspace(2*pi,0,length(t)/2+1);
            roll = [r1(1:end) r1(end)*ones(1,length(halt)) ...
                r2(1:end-1) r2(end)*ones(1,length(halt))]';
        end
        
    case 'pitch'
        yaw = zeros(length(t)+2*length(halt),1);
        roll = zeros(length(t)+2*length(halt),1);
        if deg < 360
            p1 = linspace(0,deg*pi/180,length(t)/4+1);
            p2 = linspace(deg*pi/180,-deg*pi/180,length(t)/4*2+1);
            p3 = linspace(-deg*pi/180,0,length(t)/4+1);
            pitch = [p1 p1(end)*ones(1,length(halt)) p2(1:end-1) ...
                p2(end-1)*ones(1,length(halt))  p3(2:end)]';
        else
            p1 = linspace(0,2*pi,length(t)/2+1);
            p2 = linspace(2*pi,0,length(t)/2+1);
            pitch = [p1(1:end) p1(end)*ones(1,length(halt)) ...
                p2(1:end-1) p2(end)*ones(1,length(halt))]';
        end
       
    case 'yaw'
        pitch = zeros(length(t)+2*length(halt),1);
        roll = zeros(length(t)+2*length(halt),1);
        if deg < 360
            y1 = linspace(0,deg*pi/180,length(t)/4+1);
            y2 = linspace(deg*pi/180,-deg*pi/180,length(t)/4*2+1);
            y3 = linspace(-deg*pi/180,0,length(t)/4+1);
            yaw = [y1 y1(end)*ones(1,length(halt)) y2(1:end-1) ...
                y2(end-1)*ones(1,length(halt)) y3(2:end)]';
        else
            y1 = linspace(0,2*pi,length(t)/2+1);
            y2 = linspace(2*pi,0,length(t)/2+1);
            yaw = [y1(1:end) y1(end)*ones(1,length(halt)) ...  
                y2(1:end-1) y2(end)*ones(1,length(halt))]';
        end
    case 'axis'
        theta = linspace(0,deg*pi/180,length(t));
        q0 = cos(theta/2);
        qvec = [sin(theta/2)'*axis(1), sin(theta/2)'*axis(2), sin(theta/2)'*axis(3)];
end

if style ~= 'axis'
    t = t(1):dt:(t(end)+halt(end)*2);
    att = eul2quat([unwrap(roll),unwrap(pitch),unwrap(yaw)],'XYZ');
else
    att = [q0',qvec];
    att = att./vecnorm(att')';
end

qdot = (att(2:end,:) - att(1:end-1,:))/dt;

q0 = att(:,1); q1 = att(:,2); q2 = att(:,3); q3 = att(:,4);
w = zeros(3,length(q0));
for i = 1:length(q0)-1
    w(:,i+1) = 2*[-q1(i) q0(i) -q3(i) q2(i);
                    -q2(i) q3(i) q0(i) -q1(i);
                    -q3(i) -q2(i) q1(i) q0(i)]*qdot(i,:)';
end

attMat = [att w'];