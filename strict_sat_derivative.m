function dy = strict_sat_derivative(s, M, L)
% Derivada de la saturacion suave estrictamente creciente
%
%   dy = dsat_strict(s, M, L)

    if M <= L
        error('Debe cumplirse M > L > 0');
    end

    alpha = 1/(M - L);

    dy = zeros(size(s));

    % Region central
    idx1 = abs(s) <= L;
    dy(idx1) = 1;

    % Region superior
    idx2 = s > L;
    dy(idx2) = exp(-alpha*(s(idx2) - L));

    % Region inferior
    idx3 = s < -L;
    dy(idx3) = exp(alpha*(s(idx3) + L));

end