function X_odom = odometry_with_ticks(R, LW, T, N, wheels, X_opti)
%     fprintf('R=%f LW=%f\n', R, LW)

    %% From wheels speed to robot speed
    M = R / 4 * [
         1.0,           1.0,           1.0,            1.0;
        -1.0,           1.0,          -1.0,            1.0;
        -1.0 / LW, 1.0 / LW, 1.0 / LW, -1.0 / LW
    ];
    
    % Each column is a set of wheels velocities
    deltaT = diff([0; wheels.time]);
    U = [ ...
        diff([0; wheels.front_left_pos])  ./ deltaT, ...
        diff([0; wheels.front_right_pos]) ./ deltaT, ...
        diff([0; wheels.rear_right_pos])  ./ deltaT, ...
        diff([0; wheels.rear_left_pos])   ./ deltaT ...
    ]';
    
    % Transform the velocity from counts/s to rad/s
    U = U / N * 2 * pi;
    
    % Apply the gear ratio 5:1
    U = U .* T;
    
    % Compute the robot velocity
    X_dot_odom = zeros(3, size(U, 2));
    for i = 1:size(U, 2)
        X_dot_odom(:, i) = M * U(:, i);
    end
    
    %% Runge-Kutta integration
    
    % Prepare matrixes
    X_odom = zeros(3, size(U, 2));
    
    % Set optitrack initial position
    X_odom(:, 1) = [X_opti(1, 1);  X_opti(2, 1); mean(X_opti(3, 1:20))];
    X_odom(3, 1) = X_odom(3, 1) + 0.07;

    for i = 2:size(X_dot_odom, 2)
        % Prepare the rotation matrix
        rtm = [
            cos(X_odom(3, i - 1) + X_dot_odom(3, i) * deltaT(i) / 2), -sin(X_odom(3, i - 1) + X_dot_odom(3, i) * deltaT(i) / 2), 0;
            sin(X_odom(3, i - 1) + X_dot_odom(3, i) * deltaT(i) / 2),  cos(X_odom(3, i - 1) + X_dot_odom(3, i) * deltaT(i) / 2), 0;
                                                               0,                                                     0, 1
        ];
    
        % Rotate the velocities
        X_dot_odom(:, i) = rtm * X_dot_odom(:, i);
    
        % Integrate
        X_odom(:, i) = X_odom(:, i - 1) + X_dot_odom(:, i) .* deltaT(i);
    end
end