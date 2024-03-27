% run_robot_model
%
% runs a simulation of a two wheeled robot. This script sets up
% the parameters of the robot, and the input voltages as timeseries
% The simulink model 'robot_model' is run, and the results plotted


% conversions from angle to counts and back
rad_to_counts = 3600/(2*pi);
counts_to_rad = 1/rad_to_counts;
r_actual_ft = .5; % actual wheel radius
b_actual_ft = 1; % actual robot width
Ts=.01; % sample time in seconds
%
% right wheel dynamic model parameters
%
K_r=1.5;
sigma_r=15;
%K_r=1.0;
%sigma_r=10;

%
% left wheel dynamic model parameters
%
K_l=1;
sigma_l=10;

V_left=timeseries([0 0 3 3],[0 .99 1 10]);
V_right=timeseries([0 0 0 0],[0 .99 1 10]);

out=sim('robot_model.slx')
figure(1)
clf
plot(V_left)
hold on
plot(V_right)
legend('Voltage left','Voltage right')
xlabel('Time (s)')
ylabel('Voltage')
figure(2)
clf
plot(out.Theta_Dot)
legend('Theta dot left','Theta dot right')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
figure(3)
clf
animate

