function [dx,u] = robot_dynamics_filtro_vel(t,x,A,K,Ms,Gamma,Ma,B,E)

%% Estados
q  = x(1:2);
dq = x(3:4);
theta_hat = x(5:14);
q_c = x(15:16);

q1=q(1); q2=q(2);
dq1=dq(1); dq2=dq(2);

%% =============================
%% Trayectoria deseada
  
[qd,dqd,ddqd] = desired_trajectory(t);

%% =============================
%% Perdida de efectividad
%d1 = (t < 50) + 0.2*(t >= 50 && t < 200) + 1.0*(t >= 200);
d1 = 1;
d2 = 1;

d = diag([d1 d2]); 

%% =============================
%% Dinámica robot Kelly

D = zeros(2);
D(1,1) = 2*0.01267*cos(q2)+0.32262;
D(1,2) = 0.01218+0.01267*cos(q2);
D(2,1) = D(1,2);
D(2,2) = 0.01218;

C = zeros(2);
C(1,1) = -0.01267*sin(q2)*dq2;
C(1,2) = -0.01267*sin(q2)*(dq1+dq2);
C(2,1) = 0.01267*sin(q2)*dq1;

%F = [0.274*dq1; 0.144*dq2];
F = [2.288*dq1; 0.175*dq2];

g = [(11.5078*sin(q1)+0.459587*sin(q1+q2));
     (0.459587*sin(q1+q2))];
 
%% =============================
%% Filtro velocidad

%q_bar = q - qd;

q_bar = q - qd;
vartheta = q_c + B * q_bar;
dq_c = -E * tanh(q_c + B*q_bar);
% =[-E(1)*tanh( q_c(1) + B(1)*q_bar(1) ) ; -E(2)*tanh( q_c(2) + B(2)*q_bar(2) ) ];

dq_r = reference_model(q,qd,dqd,A);
s = vartheta - dq_r;
%s = dq - dq_r;

%% ============================= 
%% Matriz Upsilon considerando \Delta diagonal
%U = regressor_U(q,dq_r,ddqd);
% Regresor dependiente de la trayectoria deseada solamente
U = regressor_U(qd,dqd,ddqd);
%% =============================
%% Saturación sigma(Ks)

%sat = @(x) max(min(x,1),-1);
%sigma = Ms*sat(K*s);
L = [0.9*Ms(1);0.9*Ms(2)];
x = K*s;

tau_p = (Ms - L).*tanh((x - L)./(Ms - L));
tau_n = (Ms - L).*tanh((x + L)./(Ms - L));

sigma = (tau_p + L).*(x > L) ...
      + x.*((x >= -L) & (x <= L)) ...
      + (tau_n - L).*(x < -L);

%% =============================
%% Control adaptable

sa = Ma.*tanh(theta_hat - Gamma*U'*q_bar);

%sa = Ma.*tanh(theta_hat);
u = d*(-sigma + U*sa);
u(1) = max(min(u(1),150),-150);
u(2) = max(min(u(2),15),-15);

%% =============================
%% Dinámica real

RHS = u - C*dq - F - g;
ddq = D \ RHS;

%% =============================
%% Ley de adaptación

theta_hat_dot = - Gamma * U' * s;

%% =============================
%% Vector derivada total

dx = zeros(14,1);
dx(1:2) = dq;
dx(3:4) = ddq;
dx(5:14) = theta_hat_dot;
dx(15:16) = dq_c;

end
