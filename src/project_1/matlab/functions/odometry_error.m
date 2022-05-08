function error = odometry_error(R, LW, T, N, wheels, X_opti)
    % odometry_error Calculates the odometry error against the reference data.
    %   Parameters:
    %       R:  Robot wheels diameter
    %       LW: Robot width plus lenght
    %       T:  Gear ratio
    %       N:  Encoder counts
    %       wheels: Wheels velocities array [4x...]
    %       X_opti: Reference positions

    % Get the initial orientation from the reference data
    start_pos = [X_opti(1, 1); X_opti(2, 1); mean(X_opti(3, 1:20))];

    % Compute odometry
    X_odom = odometry_with_ticks(R, LW, T, N, wheels, start_pos);

    % Compute distance between odometry and optitrack over position
    error = sum(sqrt(sum((X_odom(1:2, :) - X_opti(1:2, :)).^2, 1)));
end
