load("parsed_data/bag2.mat")
robot_parameters

% Best for bag2 with theta_off = 0.07
R  = 0.075340;
LW = 0.354743;

% Best for bag3 with theta_off = 0.04
% R  = 0.075854;
% LW = 0.361886;

% Downsample optitrack data
X_opti = [optitrack.x, optitrack.y, optitrack.theta]';
X_opti = resample(X_opti, size(wheels.time, 1), size(optitrack.time, 1), 'Dimension', 2);

X_odom = odometry_with_ticks(R, LW, T, N, wheels, X_opti);

figure
hold on
plot(X_opti(1, :), X_opti(2, :))
plot(X_odom(1, :), X_odom(2, :))
legend('OptiTrack', 'Odometry')