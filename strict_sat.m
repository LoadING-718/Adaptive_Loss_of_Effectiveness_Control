function y = strict_sat(s, M, L)
% SAT_STRICT  Smooth strictly increasing saturation funtion
%
%   y = sat_strict(s, M, L)
%
%   M : saturation level (M > L > 0)
%   L : linear region width
%
%   The function is:
%   - Linear between [-L, L]
%   - Saturacion exponencial fuera
%   - C1 y estrictamente creciente

    if M <= L
        error('You must chose M > L > 0');
    end

    alpha = 1/(M - L);

    y = zeros(size(s));

    % Linear region
    idx1 = abs(s) <= L;
    y(idx1) = s(idx1);

    % Superior region
    idx2 = s > L;
    y(idx2) = M - (M - L)*exp(-alpha*(s(idx2) - L));

    % Inferior region
    idx3 = s < -L;
    y(idx3) = -M + (M - L)*exp(alpha*(s(idx3) + L));

end