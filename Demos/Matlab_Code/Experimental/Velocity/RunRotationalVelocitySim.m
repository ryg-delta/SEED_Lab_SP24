%% RunRotationalVelocitySim.m
% This script runs a simulation of a motor and plots the results
%
% required file: velocitysim.slx
%
%% Define parameters
K=0.405; % DC gain [rad/Vs]
sigma=5; % time constant reciprocal [1/s]
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('rotationalVelocitySim')
%
% run the simulation
%

out=sim('rotationalVelocitySim');
%% load rotational velocity data
load('motorRotationalVelocityData.mat');
%% A Plot of the results
%
figure
subplot(2,1,1)
plot(out.deltaVa,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.rotationalVelocity,'--','linewidth',2)
hold on
plot(data(:,1),abs(data(:,3)),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Rotational Velocity (rad/s)')
