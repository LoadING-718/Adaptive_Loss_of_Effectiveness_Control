function M = myfun2(t, x, t_data)

    % Interpolar estados q(t)
    q_interp = interp1(t_data, x(:,1:2), t, 'linear', 'extrap');

    % Asegurar formato consistente
    if isvector(t)
        N = length(t);
    else
        N = 1;
    end

    M = zeros(10,10,N); % porque U es 2x10 ? U'U es 10x10

    for k = 1:N

        q = q_interp(k,:)';

        % Trayectoria deseada
        [qd,dqd,ddqd] = desired_trajectory(t(k));

        % Control auxiliar
        q_bar = q - qd;
        A = diag([100 90]);
        z = A*q_bar;
        psi_z  = psi_sat(z);
        dq_r  = dqd - psi_z;

        % Variables
        q1 = q(1);
        q2 = q(2);

        % Regresor U (2x10)
        U = [cos(q2)*(2*ddqd(1)+ddqd(2)) - dq_r(2)*(dq_r(1)+dq_r(2))*sin(q2) - dq_r(2)*dq_r(1)*sin(q2), ...
             0, ...
             ddqd(1), ...
             ddqd(2), ...
             0, ...
             dq_r(1), ...
             0, ...
             sin(q1), ...
             sin(q1+q2), ...
             0;

             0, ...
             ddqd(1)*cos(q2) + dq_r(1)^2*sin(q2), ...
             0, ...
             0, ...
             ddqd(1)+ddqd(2), ...
             0, ...
             dq_r(2), ...
             0, ...
             0, ...
             sin(q1+q2)];

        % Matriz de Gram
        M(:,:,k) = U' * U;

    end
end