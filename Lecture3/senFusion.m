% This is an example of Kalman Filter application for sensor fusion
% between acceleretion data (accelerometer) and position data (GPS)

clear;
clc;

load acceleration.txt;
acceleration = acceleration/10;

dt = 0.001;

% system description
F = [1 dt; 0 1];
G = [0.5*dt^2;dt];
H = [1 0];

% error covariance matrix
Q = 0.01*eye(2);
R = 0.01;

% initial data
x    = [10;0];
xhat = [0;0];
Pplus = 0*eye(2);

% for plotting
xArray    = [];
xhatArray = [];

for i=1:length(acceleration(:,2))
    
    xArray    = [xArray x];
    xhatArray = [xhatArray xhat];
    
    u = acceleration(i,2);
    
    x = F*x + G*u + sqrt(Q*dt)*[randn;randn];
    y = H*x + sqrt(R*dt)*randn;               % one position sensor
    
    % Prediction
    xmin  = F*xhat + G*u;
    Pmin  = F*Pplus*F' + Q;
    % Update
    Sigma = inv(H*Pmin*H' + R);
    K     = Pmin*H'*Sigma;
    Pplus = (1-K*H)*Pmin;
    xhat  = xmin + K*(y-H*xmin);
end

plot(dt:dt:length(acceleration(:,2))*dt,xArray(1,:),':b','LineWidth',3)
hold on;
plot(dt:dt:length(acceleration(:,2))*dt,xhatArray(1,:),':r','LineWidth',3)
legend('true','estimated')
set(gca,'FontSize',24)
grid on;
grid minor;