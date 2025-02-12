function attMat = aim(aimVec,posMat,trans,dt,t)

traj = posMat(:,1:3);
vel = posMat(:,4:6);

%trans = 10; %5 sec transit time
len = length(0:dt:trans);
%% Aim
% +x faces aim point
dir_vec = aimVec-traj;
dir_vec = dir_vec./vecnorm(dir_vec')';

%+z is perpendicular to velocity
dir_perp = cross(vel,dir_vec);
dir_perp = dir_perp./vecnorm(dir_perp')';

%+y is perpendicular to +x and +z
dir_y = cross(dir_perp,dir_vec);
dir_y = dir_y./vecnorm(dir_y')';

% %+y is perpendicular to +x and +z
% dir_y = -vel;
% dir_y = dir_y./vecnorm(dir_y')';
% 
% %+z is perpendicular to velocity
% dir_z = cross(dir_vec,-vel);
% dir_z = dir_z./vecnorm(dir_z')';


%% euler angles
pitch = asin(-dir_vec(:,3));
pitch = unwrap(pitch);

yaw = atan2(dir_vec(:,2),dir_vec(:,1));
yaw = unwrap(yaw);

roll = atan2(dir_y(:,3),dir_perp(:,3));
roll = unwrap(roll);

% yaw = atan2(-dir_vec(:,1), dir_vec(:,2));
% pitch = atan2(-dir_vec(:,3),sqrt(dir_vec(:,1).^2+dir_vec(:,2).^2));
% roll = zeros(size(pitch));
% pitch = unwrap(pitch);
% yaw = unwrap(yaw);
% roll = unwrap(roll);

% roll(1:len) = zeros(len,1); %linspace(0,roll(len),len);
% pitch(1:len) = zeros(len,1); %linspace(0,pitch(len),len);
% yaw(1:len) = zeros(len,1); %linspace(0,yaw(len),len);
% roll(end-len+1:end) = zeros(len,1);
% pitch(end-len+1:end) = zeros(len,1);
% yaw(end-len+1:end) = zeros(len,1);
%% Angular Velocity
euler = [yaw,pitch,roll];

att = eul2quat(euler,'ZYX');
% att([1:len,end-(len-1):end],1) = [linspace(1,att(len,1),len) ...
%     linspace(att(end-(len-1),1),1,len)];
% att([1:len,end-(len-1):end],2) = [linspace(0,att(len,2),len) ...
%     linspace(att(end-(len-1),2),0,len)];
% att([1:len,end-(len-1):end],3) = [linspace(0,att(len,3),len) ...
%     linspace(att(end-(len-1),3),0,len)];
% att([1:len,end-(len-1):end],4) = [linspace(0,att(len,4),len) ...
%     linspace(att(end-(len-1),4),0,len)];

% v = 25;
% v2 = 500;
% att([1:len],1) = interp1(t(1:v:len+v2),att(1:v:len+v2,1),t(1:len));
% att([1:len],2) = interp1(t(1:v:len+v2),att(1:v:len+v2,2),t(1:len));
% att([1:len],3) = interp1(t(1:v:len+v2),att(1:v:len+v2,3),t(1:len));
% att([1:len],4) = interp1(t(1:v:len+v2),att(1:v:len+v2,4),t(1:len));

%% Ramp In/Ramp Out
q0ss = ones(trans/dt+1,1);
qvecss = zeros(trans/dt+1,1);


%val = 50;
%trans = 10; %transit time in sec

t1 = -trans:dt:0;
t2 = trans:dt:2*trans;
ttot1 = [t1 t2];

t3 = t(end)-2*trans:dt:t(end)-trans;
t4 = t(end):dt:t(end)+trans;
ttot2 = [t3 t4];

tmeas1 = 0:dt:trans;
tmeas2 = t(end)-trans:dt:t(end);

ramp_in_end = find(t==trans);
ramp_out_start = find(t==t(end)-trans);
%start
% 
att(1:ramp_in_end,1) = spline(ttot1,[q0ss; att(find(t==t2(1)):find(t==t2(end)),1)],tmeas1)';
att(1:ramp_in_end,2) = spline(ttot1,[qvecss; att(find(t==t2(1)):find(t==t2(end)),2)],tmeas1)';
att(1:ramp_in_end,3) = spline(ttot1,[qvecss; att(find(t==t2(1)):find(t==t2(end)),3)],tmeas1)';
att(1:ramp_in_end,4) = spline(ttot1,[qvecss; att(find(t==t2(1)):find(t==t2(end)),4)],tmeas1)';
%end
att(ramp_out_start:end,1) = spline(ttot2,[att(find(t==t3(1)):find(t==t3(end)),1);q0ss],tmeas2)';
att(ramp_out_start:end,2) = spline(ttot2,[att(find(t==t3(1)):find(t==t3(end)),2);qvecss],tmeas2)';
att(ramp_out_start:end,3) = spline(ttot2,[att(find(t==t3(1)):find(t==t3(end)),3);qvecss],tmeas2)';
att(ramp_out_start:end,4) = spline(ttot2,[att(find(t==t3(1)):find(t==t3(end)),4);qvecss],tmeas2)';

%% 


% ia = find(t==35);
% ib = find(t==45);
% len2 = length(35:dt:45);
% att(ia:ib,1) = linspace(att(ia,1),att(ib,1),len2);
% att(ia:ib,2) = linspace(att(ia,2),att(ib,2),len2);
% att(ia:ib,3) = linspace(att(ia,3),att(ib,3),len2);
% att(ia:ib,4) = linspace(att(ia,4),att(ib,4),len2);



% v = 50;
% att(:,1) = interp1(t(1:v:end),att(1:v:end,1),t);
% att(:,2) = interp1(t(1:v:end),att(1:v:end,2),t);
% att(:,3) = interp1(t(1:v:end),att(1:v:end,3),t);
% att(:,4) = interp1(t(1:v:end),att(1:v:end,4),t);

%att = lowpass(att,.002,50,'steepness',.95);

% [b,a] = butter(4,0.1/25);
% 
% att(:,1) = filter(b,a,att(:,1));
% att(:,2) = filter(b,a,att(:,2));
% att(:,3) = filter(b,a,att(:,3));
% att(:,4) = filter(b,a,att(:,4));

qdot = (att(2:end,:) - att(1:end-1,:))/dt;

q0 = att(:,1); q1 = att(:,2); q2 = att(:,3); q3 = att(:,4);
w = zeros(3,length(q0));
for i = 1:length(q0)-1
    w(:,i+1) = 2*[-q1(i) q0(i) q3(i) -q2(i);
                    -q2(i) -q3(i) q0(i) q1(i);
                    -q3(i) q2(i) -q1(i) q0(i)]*qdot(i,:)';
end

attMat = [att w'];

%%
% phidot = (roll(3:end)-roll(2:end-1))/dt;
% phidot = [phidot(1)*ones(2,1); phidot];
% thetadot = (pitch(3:end)-pitch(2:end-1))/dt;
% thetadot = [thetadot(1)*ones(2,1); thetadot];
% psidot = (yaw(3:end)-yaw(2:end-1))/dt;
% psidot = [psidot(1)*ones(2,1); psidot];
% 
% w1 = phidot-sin(pitch).*psidot;
% w2 = cos(roll).*thetadot+sin(roll).*cos(pitch).*psidot;
% w3 = -sin(roll).*thetadot+cos(roll).*cos(pitch).*psidot;
% 
% figure(2)
% subplot(2,1,1)
% plot([w1,w2,w3]*180/pi)
% ylim([-30 30])
% subplot(2,1,2)
% plot(w'*180/pi)
% legend('wx','wy','wz')
% ylim([-30 30])
end
