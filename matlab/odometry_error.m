function error = odometry_error(R, LW, T, N, wheels, X_opti)
    % Compute odometry
    X_odom = odometry_with_ticks(R, LW, T, N, wheels, X_opti);

    % Compute distance between odometry and optitrack over position
%         error = sqrt(sum((X_odom(1:2, :) - X_opti(1:2, :)).^2, 1));
%         error = sqrt(sum((X_odom(3, :) - X_opti(3, :)).^2, 1));
    error = sum(sqrt(sum((X_odom(1:2, :) - X_opti(1:2, :)).^2, 1)));
end