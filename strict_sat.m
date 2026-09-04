function y = strict_sat(s, M, L)
% SAT_STRICT  Saturacion suave estrictamente creciente
%
%   y = sat_strict(s, M, L)
%
%   M : nivel de saturacion (M > L > 0)
%   L : ancho de la region lineal
%
%   La funcion es:
%   - Lineal en [-L, L]
%   - Saturacion exponencial fuera
%   - C1 y estrictamente creciente

    if M <= L
        error('Debe cumplirse M > L > 0');
    end

    alpha = 1/(M - L);

    y = zeros(size(s));

    % Region central lineal
    idx1 = abs(s) <= L;
    y(idx1) = s(idx1);

    % Region superior
    idx2 = s > L;
    y(idx2) = M - (M - L)*exp(-alpha*(s(idx2) - L));

    % Region inferior
    idx3 = s < -L;
    y(idx3) = -M + (M - L)*exp(alpha*(s(idx3) + L));

end