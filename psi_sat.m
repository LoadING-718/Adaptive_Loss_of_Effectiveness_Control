function y = psi_sat(x)
% Saturación suave y estrictamente creciente

%L = [3;3]; 
M = [1;1];
L = [0.9*M(1) ; 0.9*M(2)];

tau_p = (M - L).*tanh((x - L)./(M - L));
tau_n = (M - L).*tanh((x + L)./(M - L));

y = (tau_p + L).*(x > L) ...
      + x.*((x >= -L) & (x <= L)) ...
      + (tau_n - L).*(x < -L);

end