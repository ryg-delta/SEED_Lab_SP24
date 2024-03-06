%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
% V=7.6, set to 200PWM cycle so Vin = 7.6*200/255
%% Define motor parameters
K=1.65; % DC gain [rad/Vs]
sigma=5.4; % time constant reciprocal [1/s]
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motorsim')
%
% run the simulation
%

out=sim('motorsim');

load('rightMotorData.mat');
%% A Plot of the results
%
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
plot(data(:,1),abs(data(:,4)),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')


