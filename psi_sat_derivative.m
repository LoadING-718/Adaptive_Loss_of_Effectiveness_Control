function dyds = psi_sat_derivative(x)
% Derivada de la saturación 
M = [4;4];
L = [0.9*M(1) ; 0.9*M(2)];

dtau_p = (1 - tanh((x - L)./(M - L)).^2);
dtau_n = (1 - tanh((x + L)./(M - L)).^2);

dyds = (dtau_p).*(x > L) ...
      + 1.*((x >= -L) & (x <= L)) ...
      + (dtau_n).*(x < -L); 

end