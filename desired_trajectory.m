function [qd,dqd,ddqd] = desired_trajectory(t)
% Trayectorias PE

w = 5*pi/10;

i = (1:5)';

freq1 = i*w/5;
freq2 = (2*i-1)*w/5;

sum1  = sum(((-1).^(i+1)./i).*sin(freq1*t));
sum2  = sum(((-1).^i./i).*sin(freq2*t));

d_sum1  = sum(((-1).^(i+1)./i).*freq1.*cos(freq1*t));
d_sum2  = sum(((-1).^i./i).*freq2.*cos(freq2*t));

dd_sum1 = -sum(((-1).^(i+1)./i).*(freq1.^2).*sin(freq1*t));
dd_sum2 = -sum(((-1).^i./i).*(freq2.^2).*sin(freq2*t));

q1d = pi/8 + (2/pi^2)*sum1;
q2d = pi/4 + (2/pi^2)*sum2;

dq1d = (2/pi^2)*d_sum1;
dq2d = (2/pi^2)*d_sum2;

ddq1d = (2/pi^2)*dd_sum1;
ddq2d = (2/pi^2)*dd_sum2;

qd = [q1d; q2d];
dqd = [dq1d; dq2d];
ddqd = [ddq1d; ddq2d];

end