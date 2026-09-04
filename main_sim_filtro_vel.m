clc; clear; close all;

%% Parámetros del controlador
Mr = diag([4; 4]); % Saturación \dot q_r
A  = diag([50 50]);
%A  = diag([250 160]);
K  = diag([1.5 0.5]);      % Ganancia Ms*sigma_(Ks)      
Ms = [100;10];           % Saturación Ms*sigma_(Ks)
Gamma = diag([.2 , 0.5 , .15 , 0.25, 0.75, 0.75 , .75 , 1.15 , 1.1, 1.1]);
Ma = [1 ; 1; 1; 1; 1; 2.5; 1; 20; 2.5; 2.5];
B = diag([0.15 0.15]);
E = diag([0.5 0.5]);

  % Para guardar el torque de cada actuador


%% Condiciones iniciales
q0  = [-pi/2; -pi/3];
dq0 = [0; 0];
theta_hat0 = zeros(10,1);
q_c0 = zeros(2,1);

x0 = [q0; dq0; theta_hat0; q_c0];

%% Simulación
tspan = [0 200];

%[t,x] = ode45(@(t,x) robot_dynamics(t,x,A,K,Ms,Gamma,Ma), tspan, x0);
[t,x] = ode15s(@(t,x) robot_dynamics_filtro_vel(t,x,A,K,Ms,Gamma,Ma,B,E), tspan, x0);

%[t,x] = ode23s(@(t,x) robot_dynamics(t,x,A,K,Ms,Gamma,Ma), tspan, x0);
tau = zeros(length(t),2);

for k = 1:length(t)
    [~, u] = robot_dynamics_filtro_vel(t(k), x(k,:)', A,K,Ms,Gamma,Ma,B,E);
    tau(k,:) = max(min(u', [150 15]), [-150 -15]); % saturación
end

%% Calcular eigenvalores
lambda_min = zeros(length(t),1);

for k = 1:length(t)

    tk = t(k);
    q = x(k,1:2)';
    dq = x(k,3:4)';
    
end

%% regresor
qd_all = zeros(2,length(t));

for k = 1:length(t)

    q = x(k,1:2)';
    dq = x(k,3:4)';

    [qd,dqd,ddqd] = desired_trajectory(t(k));
    
    qd_all(:,k) = qd;

    dq_r = reference_model(q,qd,dqd,A);
    
    %U = regressor_U(q,dq_r, ddqd);
    % Regressor with desired trajectory
    U = regressor_U(qd, dqd, ddqd);

    lambda_min(k) = min(svd(U))^2;
     
end
    
%% Errores para graficar

e = x(:,1:2)' - qd_all;

e1 = e(1,:);
e2 = e(2,:);

e_norm = sqrt(e1.^2 + e2.^2);

%% Parametros para graficar
theta_hat = x(:,5:14);
%sa = Ma.*tanh(theta_hat);
dim_th = size(theta_hat);
sa = zeros(dim_th(1),10);
for i = 1:1:10
    sa(:,i) = Ma(i)*tanh( theta_hat(:,i) );
end


%% Graficas
figure;
plot(t,x(:,1:2))
legend('q1','q2')
title('Posiciones')

figure;
plot(t,e1,'LineWidth',1.5); hold on
plot(t,e2,'LineWidth',1.5);
legend('e1','e2')
title('Error de Posición')
grid on

figure;
plot(t,e_norm,'LineWidth',1.5)
title('Norma del Error')
grid on

figure;
%plot(t,theta_hat(:,5:11))
%plot(t, sa)
plot(t, sa(:,1:7))
legend('th1','th2','th3','th4','th5','th6','th7')
title('Parámetros')

figure;
plot(t, sa(:,8:10))
legend('th8','th9','th10')
title('Parámetros th8-th10')

figure
plot(t,lambda_min,'LineWidth',1.5)
title('Minimum eigenvalue of U U^T')
xlabel('t')
ylabel('\lambda_{min}')
grid on

figure;
plot(t, tau(:,1),'LineWidth',1.5); hold on;
plot(t, tau(:,2),'LineWidth',1.5);
legend('Torque 1','Torque 2');
xlabel('Tiempo [s]');
ylabel('Torque [Nm]');
grid on;

%% Código comentado que quiza recicle
%[qd,dqd,ddqd] = desired_trajectory(t);

%w = 9*pi/8;

%sum1 = 0; d_sum1 = 0; dd_sum1 = 0;
%sum2 = 0; d_sum2 = 0; dd_sum2 = 0;
% 
% for i = 1:9
%     
%     % Joint 1
%     freq1 = i*w/9;
%     
%     sum1 = sum1 + ((-1)^(i+1)/i)*sin(freq1*t);
%     d_sum1 = d_sum1 + ((-1)^(i+1)/i)*freq1*cos(freq1*t);
%     dd_sum1 = dd_sum1 - ((-1)^(i+1)/i)*(freq1^2)*sin(freq1*t);
%     
%     % Joint 2
%     freq2 = (2*i-1)*w/9;
%     
%     sum2 = sum2 + ((-1)^i/i)*sin(freq2*t);
%     d_sum2 = d_sum2 + ((-1)^i/i)*freq2*cos(freq2*t);
%     dd_sum2 = dd_sum2 - ((-1)^i/i)*(freq2^2)*sin(freq2*t);
% end
% 
% 
% qd1 = pi/8 + (2/pi^2)*sum1;
% qd2 = pi/4 + (2/pi^2)*sum2;
% 
% dqd1 = (2/pi^2)*d_sum1;
% dqd2 = (2/pi^2)*d_sum2;
% 
% ddqd1 = (2/pi^2)*dd_sum1;
% ddqd2 = (2/pi^2)*dd_sum2;

%qd = [q1d; q2d];
%dqd = [dq1d; dq2d];
%ddqd = [ddq1d; ddq2d];

    %[dq_r,ddq_r] = reference_model(q,dq,qd,dqd,ddqd,A);
    %U = regressor_U(q,dq,dq_r,ddq_r);
    %M = U*U';
    %lambda = eig(M);
    %lambda_min(k) = min(lambda);
    %lambda_min(k) = min(svd(U))^2;
    %det_val(k) = det(U*U');
    
%qd1 = sin(0.5*t);
%qd2 = cos(0.5*t);

%qd1=pi/8+(2*sin(0.125*pi*t)-sin(0.25*pi*t)+2/3*sin(0.375*pi*t)-0.5*sin(0.5*pi*t)+0.4*sin(0.625*pi*t)-1/3*sin(0.75*pi*t)+2/7*sin(0.875*pi*t))*(1/(pi^2));
%qd2=pi/4+(2*sin(0.125*pi*t)-sin(0.25*pi*t)+2/3*sin(0.375*pi*t)-0.5*sin(0.5*pi*t)+0.4*sin(0.625*pi*t)-1/3*sin(0.75*pi*t)+2/7*sin(0.875*pi*t))*(1/(pi^2));

% Otra trayectoria

 %w = 9*pi/8;
 %sum = 0;
 
 %for i=1:1:9
 %    sum = sum + ((-1)^(i+1)* sin(i*w*t/9))/i;
 %end
% 
 %qd1 = pi/8 + (2/pi^2)*sum;
 %qd2 = pi/4 + (2/pi^2)*sum;
 
 
%e1 = x(:,1) - qd1;
%e2 = x(:,2) - qd2;

 %qd  = [sin(0.5*t); cos(0.5*t)];
 %dqd = [0.5*cos(0.5*t); -0.5*sin(0.5*t)];
 %ddqd = [-0.25*sin(0.5*t); -0.25*cos(0.5*t)];


% Trayectoria 
% q1d=pi/8+(2*sin(0.125*pi*t)-sin(0.25*pi*t)+2/3*sin(0.375*pi*t)-0.5*sin(0.5*pi*t)+0.4*sin(0.625*pi*t)-1/3*sin(0.75*pi*t)+2/7*sin(0.875*pi*t))*(1/(pi^2));
% q2d=pi/4+(2*sin(0.125*pi*t)-sin(0.25*pi*t)+2/3*sin(0.375*pi*t)-0.5*sin(0.5*pi*t)+0.4*sin(0.625*pi*t)-1/3*sin(0.75*pi*t)+2/7*sin(0.875*pi*t))*(1/(pi^2));
% 
% dq1d =(0.25/pi)*(cos(0.125*pi*t)-cos(0.25*pi*t)+cos(0.375*pi*t)-cos(0.5*pi*t)+cos(0.625*pi*t)-cos(0.75*pi*t)+cos(0.875*pi*t));
% dq2d =(0.25/pi)*(cos(0.125*pi*t)-cos(0.25*pi*t)+cos(0.375*pi*t)-cos(0.5*pi*t)+cos(0.625*pi*t)-cos(0.75*pi*t)+cos(0.875*pi*t));
% 
% ddq1d =(1/32)*(-sin(0.125*pi*t)+2*sin(0.25*pi*t)-3*sin(0.375*pi*t)+4*sin(0.5*pi*t)-5*sin(0.625*pi*t)+6*sin(0.75*pi*t)-7*sin(0.875*pi*t));
% ddq2d =(1/32)*(-sin(0.125*pi*t)+2*sin(0.25*pi*t)-3*sin(0.375*pi*t)+4*sin(0.5*pi*t)-5*sin(0.625*pi*t)+6*sin(0.75*pi*t)-7*sin(0.875*pi*t));
% 
% qd = [q1d;q2d];
% dqd = [dq1d;dq2d];
% ddqd = [ddq1d;ddq2d];

% % Otra trayectoria
% w = 9*pi/8;
% sum = 0;
% d_sum = 0;
% dd_sum = 0;
% 
% for i=1:9
%     sum = sum + ((-1)^(i+1)* sin(i*w*t/9))/i;
%     d_sum = d_sum + ((-1)^(i+1)* cos(i*w*t/9))*w/9;
%     dd_sum = dd_sum - ((-1)^(i+1)* sin(i*w*t/9))*w*w*i/81;
% end
% 
% q1d = pi/8 + (2/pi^2)*sum;
% q2d = pi/4 + (2/pi^2)*sum;
% 
% dq1d = (2/pi^2)*d_sum;
% dq2d = (2/pi^2)*d_sum;
% 
% ddq1d = (2/pi^2)*dd_sum;
% ddq2d = (2/pi^2)*dd_sum;
% 
%  qd = [q1d;q2d];
%  dqd = [dq1d;dq2d];
%  ddqd = [ddq1d;ddq2d];
% 
% % Otra trayectoria 2
% w = 9*pi/8;
% 
% sum1 = 0; d_sum1 = 0; dd_sum1 = 0;
% sum2 = 0; d_sum2 = 0; dd_sum2 = 0;
% 
% for i = 1:9
%     
%     % Joint 1
%     freq1 = i*w/9;
%     
%     sum1 = sum1 + ((-1)^(i+1)/i)*sin(freq1*t);
%     d_sum1 = d_sum1 + ((-1)^(i+1)/i)*freq1*cos(freq1*t);
%     dd_sum1 = dd_sum1 - ((-1)^(i+1)/i)*(freq1^2)*sin(freq1*t);
%     
%     % Joint 2
%     freq2 = (2*i-1)*w/9;
%     
%     sum2 = sum2 + ((-1)^i/i)*sin(freq2*t);
%     d_sum2 = d_sum2 + ((-1)^i/i)*freq2*cos(freq2*t);
%     dd_sum2 = dd_sum2 - ((-1)^i/i)*(freq2^2)*sin(freq2*t);
% 
% end
% 
% q1d = pi/8 + (2/pi^2)*sum1;
% q2d = pi/4 + (2/pi^2)*sum2;
% 
% dq1d = (2/pi^2)*d_sum1;
% dq2d = (2/pi^2)*d_sum2;
% 
% ddq1d = (2/pi^2)*dd_sum1;
% ddq2d = (2/pi^2)*dd_sum2;
% 
% qd = [q1d; q2d];
% dqd = [dq1d; dq2d];
% ddqd = [ddq1d; ddq2d];

% \dot q_r con tanh()
%dq_r = Mr * tanh(dqd - A*q_bar);
%ddq_r = Mr * (1 - tanh(dqd - A*q_bar).^2).* (ddqd - A*(dq - dqd));

% Con region lineal
%z = A*q_bar;                % argumento de saturación
%psi_z = psi_sat(z);            % salida saturada
%dpsi_z = psi_sat_derivative(z);% derivada

%dq_r = dqd - psi_z;                % velocidad
%ddq_r = ddqd - dpsi_z .* (A*(dq - dqd)); % aceleración

% Con \dot q_d fuera de la saturacion
%dq_r = dqd - Mr * tanh(A*q_bar);
%ddq_r = ddqd - Mr * (1 - tanh(A*q_bar).^2).* (A*(dq - dqd));
%% Matriz de regresión Upsilon sin \Delta
% 
% U = [ddq_r(1), ...
%      cos(q2)*(2*ddq_r(1)+ddq_r(2)) - dq_r(2)*(dq(1)+dq(2))*sin(q2) - dq(2)*dq_r(1)*sin(q2), ...
%      ddq_r(2), ...
%      dq_r(1), ...
%      0, ...
%      sin(q1), ...
%      sin(q1+q2);
%      
%      0, ...
%      ddq_r(1)*cos(q2) + dq(1)*dq_r(1)*sin(q2), ...
%      ddq_r(1)+ddq_r(2), ...
%      0, ...
%      dq_r(2), ...
%      0, ...
%      sin(q1+q2)];
% 
% U = [cos(q2)*(2*ddq_r(1)+ddq_r(2)) - dq_r(2)*(dq(1)+dq(2))*sin(q2) - dq(2)*dq_r(1)*sin(q2), ...
% 	 0, ...
% 	 ddq_r(1), ...
%      ddq_r(2), ...
% 	 0, ...
%      dq_r(1), ...
%      0, ...
%      sin(q1), ...
%      sin(q1+q2);
%      
%      0, ...
%      ddq_r(1)*cos(q2) + dq(1)*dq_r(1)*sin(q2), ...
% 	 0, ...
% 	 0, ...
%      ddq_r(1)+ddq_r(2), ...
%      0, ...
%      dq_r(2), ...
%      0, ...
%      sin(q1+q2)];

%U = regressor_U(q,dq,qd,dqd,ddqd,A);
%sa = eye(9,1);

%for i = 1:1:9
%    sa(i) = Ma(i)*tanh( theta_hat(i) );
%end

 




