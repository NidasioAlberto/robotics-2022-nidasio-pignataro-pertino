load("parsed_data/bag2.mat")
robot_parameters

% Trajectory
figure
axis equal
plot(optitrack.x, optitrack.y)
grid on

% RPM
figure
hold on
plot(wheels.time, wheels.front_left_speed, '-')
plot(wheels.time, wheels.front_right_speed, '-')
plot(wheels.time, wheels.rear_left_speed, '-')
plot(wheels.time, wheels.rear_right_speed, '-')
grid on
legend('front left speed', 'front right speed', 'rear left speed', 'rear right speed')

% Encoders
fl_enc = diff([0; wheels.front_left_pos(:)])./diff([0; wheels.time(:)]);
fl_enc = fl_enc / N * 2 * pi;

figure
hold on
plot(wheels.time, fl_enc, '-x')
plot(wheels.time, wheels.front_left_speed / 60, '-x')
legend('front left speed from rpm', 'front left speed from encoders')