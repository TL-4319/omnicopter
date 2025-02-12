%% Load Data
clc; clear all; close all;
[file_name, path] = uigetfile('.ulg');
file = [path, file_name];

ulog = ulogreader(file);
msg = readTopicMsgs(ulog);
params = readParameters(ulog);
loggedoutput = readLoggedOutput(ulog);
d1 = ulog.StartTime; d2 = ulog.EndTime;
%% Parse the data
data = readTopicMsgs(ulog, 'TopicNames', {'actuator_outputs', 'omni_adap_debug', 'vehicle_visual_odometry' }, 'InstanceID', {2, 0, 0}, 'Time', [d1, d2]);

actuator_outputs = data.TopicMessages{1,1};
omni_debug = data.TopicMessages{2,1};
vehicle_visual_odometry = data.TopicMessages{3,1};

omni_debug.timestamp = omni_debug.timestamp - d1;
vehicle_visual_odometry.timestamp = vehicle_visual_odometry.timestamp - d1;
actuator_outputs.timestamp = actuator_outputs.timestamp - d1;
%% Data Processing
[r, p, y] = quat2angle(omni_debug.attq, 'XYZ');
[r_sp, p_sp, y_sp] = quat2angle(omni_debug.attq_sp, 'XYZ');

a = 1/2 + 1/sqrt(12); b = 1/2 - 1/sqrt(12); c = 1/sqrt(3);
K = 1.90e-2;
K = diag([K,K,K,K,-K,-K,-K,-K]);
J = 0.19;
N = [-a  b  -b  a  a  -b  b  -a;...
     b   a  -a -b -b  -a  a   b;...
     c  -c  -c  c  c  -c  -c  c];
P = 1/sqrt(3)*[1 -1 1 -1 1 -1 1 -1;...
               1 1 -1 -1 1 1 -1 -1;...
               1 1 1 1 -1 -1 -1 -1];

B = [N; J*cross(P, N) + N*K];
B_pinv = pinv(B);

f_rot_act = (B_pinv*[omni_debug.f_cmd, omni_debug.t_cmd]')';

%% Plotting
close all
figure(1)
subplot(6,1,1)
plot(omni_debug.timestamp, r*180/pi, omni_debug.timestamp, r_sp*180/pi)
legend('actual','sp')
ylabel('roll, deg')
subplot(6,1,2)
plot(omni_debug.timestamp, omni_debug.omega_cmd(:,1), omni_debug.timestamp, omni_debug.t_cmd(:,1));
legend('wx','tx')
ylabel('commands')

subplot(6,1,3)
plot(omni_debug.timestamp, p*180/pi, omni_debug.timestamp, p_sp*180/pi)
legend('actual','sp')
ylabel('pitch, deg')
subplot(6,1,4)
plot(omni_debug.timestamp, omni_debug.omega_cmd(:,2), omni_debug.timestamp, omni_debug.t_cmd(:,2));
legend('wy','ty')
ylabel('commands')

subplot(6,1,5)
plot(omni_debug.timestamp, y*180/pi, omni_debug.timestamp, y_sp*180/pi)
legend('actual','sp')
ylabel('yaw, deg')
subplot(6,1,6)
plot(omni_debug.timestamp, omni_debug.omega_cmd(:,3), omni_debug.timestamp, omni_debug.t_cmd(:,3));
legend('wz','tz')
ylabel('commands')
subtitle('Attitude')

figure(2);
subplot(3,1,1)
plot(omni_debug.timestamp, omni_debug.pos(:,1), vehicle_visual_odometry.timestamp, vehicle_visual_odometry.y)
ylabel('x, m')
subplot(3,1,2)
plot(omni_debug.timestamp, omni_debug.pos(:,2), vehicle_visual_odometry.timestamp, vehicle_visual_odometry.x)
ylabel('y,m')
subplot(3,1,3)
plot(omni_debug.timestamp, omni_debug.pos(:,3), vehicle_visual_odometry.timestamp, -vehicle_visual_odometry.z)
legend('debug','odom')
ylabel('z, m')
subtitle('Position')

figure(3);
for i=1:4
    subplot(4,2,2*i-1)
    plot(actuator_outputs.timestamp, actuator_outputs.output(:,i)); title(['Motor ', num2str(i)]); ylim([900, 2100]); yline(1500, 'k--')
    subplot(4,2,2*i)
    plot(actuator_outputs.timestamp, actuator_outputs.output(:,i+4)); title(['Motor ', num2str(i+4)]); ylim([900, 2100]); yline(1500, 'k--')
end


ti = seconds(omni_debug.timestamp(1));
tf = seconds(omni_debug.timestamp(end));
Xticks = seconds([0:2:tf]);

figure(4)
subplot(5,1,1)
plot(omni_debug.timestamp, omni_debug.f_cmd), legend('f_x', 'f_y', 'f_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Force [N]')
subplot(5,1,2)
plot(omni_debug.timestamp, omni_debug.t_cmd), legend('\tau_x', '\tau_y', '\tau_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Torque [Nm]')
subplot(5,1,3:4)
plot(omni_debug.timestamp, omni_debug.f_rot_cmd(:,1:4), omni_debug.timestamp, omni_debug.f_rot_filt(:,1:4)), legend('f_r_o_t_, _1', 'f_r_o_t_, _2', 'f_r_o_t_, _3', 'f_r_o_t_, _4'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel('Force [N]')
yline(0, 'k--')
hold on; plot(omni_debug.timestamp, omni_debug.f_rot_lim(:,1), 'k--', omni_debug.timestamp, -omni_debug.f_rot_lim(:,1), 'k--'); 
subplot(5,1,5)
plot(omni_debug.timestamp, omni_debug.phi(:,1)); set(gca, 'XTick', Xticks), xlim(seconds([ti,tf]))

figure(5)
subplot(5,1,1)
plot(omni_debug.timestamp, omni_debug.f_cmd), legend('f_x', 'f_y', 'f_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Force [N]')
subplot(5,1,2)
plot(omni_debug.timestamp, omni_debug.t_cmd), legend('\tau_x', '\tau_y', '\tau_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Torque [Nm]')
subplot(5,1,3:4)
plot(omni_debug.timestamp, omni_debug.f_rot_cmd(:,5:8), omni_debug.timestamp, omni_debug.f_rot_filt(:,5:8)), legend('f_r_o_t_, _5', 'f_r_o_t_, _6', 'f_r_o_t_, _7', 'f_r_o_t_, _8'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel('Force [N]')
yline(0, 'k--')
hold on; plot(omni_debug.timestamp, omni_debug.f_rot_lim(:,1), 'k--', omni_debug.timestamp, -omni_debug.f_rot_lim(:,1), 'k--');
subplot(5,1,5)
plot(omni_debug.timestamp, omni_debug.phi(:,2)); set(gca, 'XTick', Xticks), xlim(seconds([ti,tf]))

figure(6)
ha = tight_subplot(2,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, omni_debug.omega*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular Velocity [°/s]', legend('roll speed', 'pitch speed', 'yaw speed')
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.t_cmd); set(gca, 'XTick', Xticks), ylabel 'Body Torque [N-m]', legend('\tau_x', '\tau_y', '\tau_z')

figure(7);
ha = tight_subplot(3,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, r*180/pi, omni_debug.timestamp, r_sp*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), title 'Roll', ylabel 'Angle [°]'
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.omega(:,1)*180/pi, omni_debug.timestamp, omni_debug.omega_sp(:,1)*180/pi); legend('\omega', '\omega_s_p'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular rate [°/s]'
axes(ha(3))
plot(omni_debug.timestamp, omni_debug.t_cmd(:,1)), set(gca, 'XTick', Xticks), ylabel 'Torque [N-m]'

figure(8);
ha = tight_subplot(3,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, p*180/pi, omni_debug.timestamp, p_sp*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), title 'Pitch', ylabel 'Angle [°]'
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.omega(:,2)*180/pi, omni_debug.timestamp, omni_debug.omega_sp(:,2)*180/pi); legend('\omega', '\omega_s_p'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular rate [°/s]'
axes(ha(3))
plot(omni_debug.timestamp, omni_debug.t_cmd(:,2)), set(gca, 'XTick', Xticks), ylabel 'Torque [N-m]'

figure(9);
ha = tight_subplot(3,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, y*180/pi, omni_debug.timestamp, y_sp*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), title 'Yaw', ylabel 'Angle [°]'
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.omega(:,3)*180/pi, omni_debug.timestamp, omni_debug.omega_sp(:,3)*180/pi); legend('\omega', '\omega_s_p'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular rate [°/s]'
axes(ha(3))
plot(omni_debug.timestamp, omni_debug.t_cmd(:,3)), set(gca, 'XTick', Xticks), ylabel 'Torque [N-m]'

%% Plot Testing




figure(10)
tiledlayout(3,1)

nexttile
plot(omni_debug.timestamp, omni_debug.x(:,1:3))
hold on
plot(omni_debug.timestamp,omni_debug.xr(:,1:3),'--')
plot(omni_debug.timestamp,omni_debug.pos,'-.')
legend('x','y','z','xr','yr','zr','x','y','z')
nexttile
plot(omni_debug.timestamp, omni_debug.x(:,4:6))
hold on
plot(omni_debug.timestamp,omni_debug.xr(:,4:6),'--')
legend('vx','vy','vz','vxr','vyr','vzr')
nexttile
plot(omni_debug.timestamp, omni_debug.x(:,7:9))
hold on
plot(omni_debug.timestamp,omni_debug.xr(:,7:9),'--')
legend('ex','ey','ez','exr','eyr','ezr')

figure(11)
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_pid_one,2,2))
hold on
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_pid_two,2,2))
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_pid_three,2,2))
legend('col1','col2','col3')

%%
m = 1.172; %kg
Tpos = 0.325; %s, position control time constant
Tpos_i = 1.33; %s, position control integral time constant
zeta_pos = 1.0; %position control damping ratio

%baseline model: position controller
Ap = [zeros(3), eye(3); zeros(3,6)]; %This should only be the part of the A matrix and B matrix for what is tracked???
Bp = 1/m*[zeros(3); eye(3)]; %1/m? I think

%augmented model
A = [Ap zeros(6,3); eye(3,6) zeros(3)];
B = [Bp; zeros(3)];
Bref = [zeros(6,3); - eye(3)]; %works for the attitude controller as well

Kp = 1/Tpos^2+(2*zeta_pos)/(Tpos*Tpos_i);
Kd = 2*zeta_pos/Tpos+1/Tpos_i;
Ki = 1/(Tpos^2*Tpos_i);
Kpid = m*[Kp*eye(3); Kd*eye(3); Ki*eye(3)];

ArefPID = A-B*Kpid';

%Adaptive Parameters
GammaPID = 200*diag([1,1,1]); %gain

Qpid = diag([1 1 1 1 1 1 1 1 1]); %Lyap parameters
Ppid = lyap(ArefPID',Qpid);

for i = 1:length(omni_debug.x(:,1))
f_bl(1:3,i) = -Kpid'*(omni_debug.x(i,:)'-[omni_debug.pos_sp(i,:)';zeros(6,1)]);
e(1:9,i) = omni_debug.x(i,:)'-[omni_debug.pos_sp(i,:)';zeros(6,1)];
end


figure(12)
plot(1:length(omni_debug.x(:,1)),f_bl)

figure(13)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,e(1:3,:))
hold on 
plot(omni_debug.timestamp, f_bl(3,:))
ylim([-0.5 0.5])
legend('ex','ey','ez')
nexttile
plot(omni_debug.timestamp,e(4:6,:))
hold on 
plot(omni_debug.timestamp, f_bl(3,:))
ylim([-0.5 2])
legend('evx','evy','evz')
nexttile
plot(omni_debug.timestamp,e(7:9,:))
hold on 
plot(omni_debug.timestamp, f_bl(3,:))
ylim([-0.5 0.5])
legend('evxI','evyI','evzI')

%%
figure(14)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,omni_debug.fbl)
legend('fblx','fbly','fblz')
nexttile
plot(omni_debug.timestamp,omni_debug.x(:,1:3)-omni_debug.xr(:,1:3))
legend('ea','eay','eaz')
nexttile
plot(omni_debug.timestamp,omni_debug.fad)
legend('fadx','fady','fadz')
ylim([-100 200])

figure(15)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,omni_debug.xdbar(:,1:3))
legend('xdx','xdy','xdz')
nexttile
plot(omni_debug.timestamp,omni_debug.xdbar(:,4:6))
legend('xdvx','xdvy','xdvz')
nexttile
plot(omni_debug.timestamp,omni_debug.xdbar(:,7:9))
legend('xdex','xdey','xdez')
%ylim([-100 200])

figure(16)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,omni_debug.xd(:,1:3))
legend('xdx','xdy','xdz')
nexttile
plot(omni_debug.timestamp,omni_debug.xd(:,4:6))
legend('xdvx','xdvy','xdvz')
nexttile
plot(omni_debug.timestamp,omni_debug.xd(:,7:9))
legend('xdex','xdey','xdez')


