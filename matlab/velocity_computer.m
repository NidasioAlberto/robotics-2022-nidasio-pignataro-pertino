load("parsed_data/bag3.mat")
robot_parameters

M = R / 4 * [
     1.0,           1.0,           1.0,            1.0;
    -1.0,           1.0,          -1.0,            1.0;
    -1.0 / (L + W), 1.0 / (L + W), 1.0 / (L + W), -1.0 / (L + W)
];

% Each column is a set of wheels velocities
U = [wheels.front_left_speed, wheels.front_right_speed, wheels.rear_right_speed, wheels.rear_left_speed]';
U = U ./ 60 .* T;

% Compute robot velocities from wheels velocities, [x; y; theta]
% Each column is a data item
X_dot_odom = zeros(3, size(U, 2));
for i = 1:size(U, 2)
    X_dot_odom(:, i) = M * U(:, i);
end

% Compute robot velocities from optitrack, [x; y; theta]
% Each column is a data item
X_dot_opti = [ ...
    diff([0; optitrack.x]) ./ diff([0; optitrack.time]), ...
    diff([0; optitrack.y]) ./ diff([0; optitrack.time]), ...
    diff([0; optitrack.theta]) ./ diff([0; optitrack.time]) ...
]';

figure
hold on
% X axis
plot(wheels.time, X_dot_odom(1, :))
plot(optitrack.time, medfilt1(movmean(X_dot_opti(1, :), 5), 100))
% Y axis
plot(wheels.time, X_dot_odom(2, :))
plot(optitrack.time, medfilt1(movmean(X_dot_opti(2, :), 5), 100))
grid on
legend( ...
    'Robot x velocity from RPM', 'Robot x velocity from OptiTrack', ...
    'Robot y velocity from RPM', 'Robot y velocity from OptiTrack' ...
)