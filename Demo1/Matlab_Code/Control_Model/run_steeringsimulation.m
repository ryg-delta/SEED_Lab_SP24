% run_steeringsimulation
%
% runs a simulation of a two wheeled robot with a forward motion and anglual
% motion controller. This script sets up
% the parameters of the robot, and the input voltages as timeseries
% The simulink model 'steeringsimulation' is run, and the results plotted


% conversions from angle to counts and back
rad_to_counts = 3200/(2*pi);
counts_to_rad = 1/rad_to_counts;
r_measured_ft = 0.2427; % measured wheel radius;
b_measured_ft = .8858; % measured robot width;
r_actual_ft = 0.2427; % actual wheel radius
b_actual_ft = .8858; % actual robot width

Unstable_V = 1.0827;

V_max = 7.8 - Unstable_V;


Ts=.01; % sample time in seconds
%
% right wheel parameters
%
K_r=1.65;
sigma_r=5.4;
%K_r=1.0;
%sigma_r=10;

%
% left wheel parameters
%
K_l=1.55;
sigma_l=5.5;

phi_d=timeseries([pi/2 pi/2],[0 15]);
rho_d=timeseries([0 0 3 3],[0 4.9 5 15]);


out=sim('steeringsimulation.slx');
figure(1)
clf
plot(phi_d)
hold on
plot(rho_d)
plot(out.phi)
plot(out.rho)
set(gca,'fontsize',14)
legend('phi setpoint','rho setpoint','phi','rho','location','northwest')
xlabel('Time (s)')
ylabel('magnitude')
title('Wheel position')
figure(2)
clf
plot(out.Pos)
set(gca,'fontsize',14)
xlabel('Time (s)')
ylabel('Position (ft)')
legend('X','Y','Phi')
title('Robot Position')
figure(3)
animate