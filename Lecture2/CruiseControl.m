% This is an example of Kalman Filter application velocity estimation

clear;
clc;

dt = 0.001;
tf = 10;

b = 50;
m = 1000;

% system description
F = 1-(b/m)*dt;
G = dt/m;
H = 1; 

u = 10000;

% error covariance matrix
Q = 0.01;
R = 0.01;

% initial data
x    = 0;
xhat = 10;
Pplus = 1;

% for plotting
xArray    = [];
xhatArray = [];

for i=1:tf/dt
    
    xArray    = [xArray x];
    xhatArray = [xhatArray xhat];

    % Simulate the system
    x = F*x + G*u + sqrt(Q)*randn*dt;
    y = H*x + sqrt(R)*randn*dt;

    % Prediction
    xmin  = F*xhat + G*u;
    Pmin  = F*Pplus*F' + Q;
    % Update
    K     = Pmin*H'*inv(H*Pmin*H' + R);
    Pplus = (1-K*H)*Pmin;
    xhat  = xmin + K*(y-H*xmin);
end

plot(dt:dt:tf,xArray,'-b','LineWidth',3)
hold on;
plot(dt:dt:tf,xhatArray,':r','LineWidth',3)
legend('true state','estimated state')
set(gca,'FontSize',24)
grid on;
grid minor;