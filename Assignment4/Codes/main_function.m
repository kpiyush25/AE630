%% Simulation
clc; clear all; close all;
m = 2;
g = 9.8;
dt = 0.01;
num_iter = 2000;
time = 0.01:dt:num_iter*dt;

J = [0.01 0    0;
     0    0.05 0;
     0    0    0.001];

wn_pos = 0.8;
xi=0.7;

phi_max = 1.436; 
theta_max = 1.436; 

desired_pos = [0.9; 0; 0]; % position desired
desired_att = [0.4; -0.2; 0];
desired_psi = desired_att(3);

pos = [0; 0; 0];
att = [0; 0; 0];

vel = [0; 0; 0];
w = [0; 0; 0]; %body rates or body axis angular rate vector
error_phi = zeros(1,num_iter); error_theta = zeros(1,num_iter); error_psi = zeros(1,num_iter);
error_x = zeros(1,num_iter); error_y = zeros(1,num_iter); error_z = zeros(1,num_iter);

for j = 1:num_iter
    error_phi(j) = att(1) - desired_att(1);
    error_theta(j) = att(2) - desired_att(2);
    error_psi(j) = att(3) - desired_att(3);
    error_x(j) = pos(1) - desired_pos(1);
    error_y(j) = pos(2) - desired_pos(2);
    error_z(j) = pos(3) - desired_pos(3);
    
    [Thrust, desired_phi, desired_theta] = position_controller(desired_pos, pos, att, vel, w, m, wn_pos ,xi, g, phi_max, theta_max);
     
    desired_att = [desired_phi; desired_theta; desired_psi];

    [l, m, n] = attitude_controller(att,desired_att, w);

    Moments = [l;m;n];
    [att, w, pos, vel] = forward_dynamics(att, w, pos, vel, Thrust, Moments, dt, J, m, g);
    
end

plot(time,error_phi,'r');
hold on;
plot(time,error_theta,'g');
hold on;
plot(time,error_psi,'b');
hold on;
figure(2);
plot(time,error_x,'b');
hold on;
plot(time,error_y,'g');
hold on;
plot(time,error_z,'r');
hold on;
