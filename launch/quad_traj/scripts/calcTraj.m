clear all; close all; clc;

M = csvread('halo_aimdown_50Hz_accel');

%% Position
pos = M(1:20,1:3);
pos(:,3) = pos(:,3)+.25;
dt = 1/50;
tread = 0:dt:(size(pos,1)-1)*dt;

p0 = [0, 0, 0.75];
transit = 5; %sec

tstart = [0 tread+transit];

xstart = [p0(1);pos(:,1)];
ystart = [p0(2);pos(:,2)];
zstart = [p0(3);pos(:,3)];

tfind = 0:dt:transit;

xnew = interpn(tstart,xstart,tfind,'spline');
ynew = interpn(tstart,ystart,tfind,'spline');
znew = interpn(tstart,zstart,tfind,'spline');

vx = (xnew(2:end)-xnew(1:end-1))/dt;
vy = (ynew(2:end)-ynew(1:end-1))/dt;
vz = (znew(2:end)-znew(1:end-1))/dt;

ax = (vx(2:end)-vx(1:end-1))/dt;
ay = (vy(2:end)-vy(1:end-1))/dt;
az = (vz(2:end)-vz(1:end-1))/dt;

v = (M(2:end,1:3)-M(1:end-1,1:3))/dt;

figure(1)
plot3(M(:,1),M(:,2),M(:,3)+.25)
hold on
plot3(xnew,ynew,znew)
xlabel('x')
ylabel('y')
zlabel('z')

posMat = [xnew',ynew',znew',[0;0;ax'],[0;0;ay'],[0;0;az']];
%%
figure(2)
tiledlayout(3,2)

nexttile(1)
plot(tfind,xnew)
hold on
plot(tread+transit,pos(:,1))

nexttile(3)
plot(tfind,ynew)
hold on
plot(tread+transit,pos(:,2))

nexttile(5)
plot(tfind,znew)
hold on
plot(tread+transit,pos(:,3))

nexttile(2)
plot(tfind(2:end),vx)
hold on
plot(tread(2:end)+transit,v(1:19,1))

nexttile(4)
plot(tfind(2:end),vy)
hold on
plot(tread(2:end)+transit,v(1:19,2))

nexttile(6)
plot(tfind(2:end),vz)
hold on
plot(tread(2:end)+transit,v(1:19,3))

%% Attitude
att = M(1:20,7:10);

q0 = [1,0, 0, 0];

qwstart = [q0(1);att(:,1)];
qxstart = [q0(2);att(:,2)];
qystart = [q0(3);att(:,3)];
qzstart = [q0(4);att(:,4)];

qwnew = interpn(tstart,qwstart,tfind,'spline');
qxnew = interpn(tstart,qxstart,tfind,'spline');
qynew = interpn(tstart,qystart,tfind,'spline');
qznew = interpn(tstart,qzstart,tfind,'spline');

q = [qwnew',qxnew',qynew',qznew'];
q = q./vecnorm(q,2,2);

qdot = (q(2:end,:) - q(1:end-1,:))/dt;

q0 = q(:,1); q1 =q(:,2); q2 = q(:,3); q3 = q(:,4);
w = zeros(3,length(q0));
for i = 1:length(q0)-1
    w(:,i+1) = 2*[-q1(i) q0(i) -q3(i) q2(i);
                    -q2(i) q3(i) q0(i) -q1(i);
                    -q3(i) -q2(i) q1(i) q0(i)]*qdot(i,:)';
end

attMat = [q w'];

figure(3)
tiledlayout(4,1)

nexttile(1)
plot(tfind,q(:,1))
hold on
plot(tread+transit,att(:,1))


nexttile(2)
plot(tfind,q(:,2))
hold on
plot(tread+transit,att(:,2))

nexttile(3)
plot(tfind,q(:,3))
hold on
plot(tread+transit,att(:,3))

nexttile(4)
plot(tfind,q(:,4))
hold on
plot(tread+transit,att(:,4))



