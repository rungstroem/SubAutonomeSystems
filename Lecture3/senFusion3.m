% This is an example of Kalman Filter application for sensor fusion
% between acceleretion data (accelerometer) and 3 position data (GPS)

clear;
clc;

load acceleration.txt;
acceleration = acceleration/10;

dt = 0.001;

% system description
F = [1 dt; 0 1];
G = [0.5*dt^2;dt];
H = [1 0;1 0;1 0];         % using 3 position sensors

% error covariance matrix
Q = 0.01*eye(2);
R = [0.01 0 0;0 0.04 0;0 0 16]; % using 3 position sensors

% initial data
x    = [0;0];
xhat = [0;0];
Pplus = eye(2);

% for plotting
xArray    = [];
xhatArray = [];

for i=1:length(acceleration(:,2))
    
    xArray    = [xArray x];
    xhatArray = [xhatArray xhat];
    
    u = acceleration(i,2);
    
    x = F*x + G*u + sqrt(Q*dt)*[randn;randn];
    y = H*x + sqrt(R*dt)*[randn;randn;randn];    % 3 position sensors
    
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