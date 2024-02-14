%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
% Make sure step input == desired speed in siumlink

K=4; % DC gain [rad/Vs]
sigma=12; % time constant reciprocal [1/s]
Kp = 4.5;  % proportional gain
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motor_control')
%
% run the simulation
%

out=sim('motor_control');
%% A Plot of the results simulated and gathered from Arduino
% array 'data' is created using ReadFromArduino.mlx
% data is format: time, voltage, velocity
figure
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'--','linewidth',2)
hold on
plot(data(:,1),abs(data(:,3)),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
