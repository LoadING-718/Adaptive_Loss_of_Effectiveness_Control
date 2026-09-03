function y = max_U

%% Bounds desired trajectory derivative
 
w = 5*pi/8;

i = (1:3)';

freq1 = i*w/10;
freq2 = (2*i-1)*w/10;

sum1  = sum(((1).^(i+1)./i) );
sum2  = sum(((1).^i./i) );

d_sum1  = sum(((1).^(i+1)./i).*freq1);
d_sum2  = sum(((1).^i./i).*freq2 );

dd_sum1 = sum(((1).^(i+1)./i).*(freq1.^2));
dd_sum2 = sum(((1).^i./i).*(freq2.^2));

q1d = pi/8 + (2.5)*sum1;
q2d = pi/4 + (2.5)*sum2;

B_dq1d = (2.5)*d_sum1;
B_dq2d = (2.5)*d_sum2;

B_ddq1d = (2.5)*dd_sum1;
B_ddq2d = (2.5)*dd_sum2;
 

%% Bounds regressor

Mr = [0.8,0.8];

%B_dqr = [0.716 + Mr(1), 1.202 + Mr(2)];
B_dqr = [B_dq1d + Mr(1), B_dq2d + Mr(2)];
B_ddqd = [B_ddq1d , B_ddq2d];
d = [0.4, 1];

U1 = 2 * B_ddqd(1) + B_ddqd(2) + B_dqr(2)*(B_dqr(1) + B_dqr(2)) + B_dqr(2)*B_dqr(1)...
     + 0 ...
     + B_ddqd(1) ...
     + B_ddqd(2) ...
     + 0 ...
     + B_dqr(1) ...
     + 0 ...
     + 1 ...
     + 1 ...
     + 0;

U2 = 0 ...
     + B_ddqd(1) + B_dqr(1)*B_dqr(1) ...
     + 0 ...
     + 0 ...
     + B_ddqd(1) + B_ddqd(2) ...
     + 0 ...
     + B_dqr(2) ...
     + 0 ...
     + 0 ...
     + 1;
 
 
 y = [U1/d(1) ; U2/d(2); B_dq1d; B_dq2d; B_ddq1d; B_ddq2d];
