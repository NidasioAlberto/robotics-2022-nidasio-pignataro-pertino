load("parsed_data/bag3.mat")
addpath('functions/')
robot_parameters

% Best parameters
best_R = 0.075797;
best_LW = 0.358886;

% Downsample optitrack data
X_opti = [optitrack.x, optitrack.y, optitrack.theta]';
X_opti = resample(X_opti, size(wheels.time, 1), size(optitrack.time, 1), 'Dimension', 2);

% Get the initial orientation from the reference data
start_pos = [X_opti(1, 1); X_opti(2, 1); mean(X_opti(3, 1:20))];

% Compute integration
X_odom = odometry_with_ticks(R, L + W, T, N, wheels, start_pos);
X_odom_best = odometry_with_ticks(best_R, best_LW, T, N, wheels, start_pos);

% Plot position
figure
hold on
plot(X_opti(1, :), X_opti(2, :), 'LineWidth', 2)
plot(X_odom(1, :), X_odom(2, :), 'LineWidth', 2)
plot(X_odom_best(1, :), X_odom_best(2, :), 'LineWidth', 2)
axis equal
grid on
xlabel('x [m]')
ylabel('y [m]')
legend('OptiTrack', 'Odometry nominal', 'Odometry calibrated')

% Plot orientation
figure
hold on
plot(X_opti(3, :), 'LineWidth', 2)
plot(X_odom(3, :), 'LineWidth', 2)
plot(X_odom_best(3, :), 'LineWidth', 2)
grid on
xlabel('samples')
ylabel('theta [rad]')
legend('OptiTrack', 'Odometry nominal', 'Odometry calibrated')
