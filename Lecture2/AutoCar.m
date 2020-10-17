% This is an example of Kalman Filter application velocity estimation

clear;
clc;

dt = 0.001;
tf = 10;

at    = 0;
lf    = 1;
delta = 0;

% system description
H = [1 0 0 0;
     0 1 0 0]; 

% error covariance matrix
Q = 0.01*eye(4);
R = 0.01*eye(2);

% initial data
x     = [0 0 0 10]';
xhat  = [0 0 0 0]';
Pplus = 1*eye(4);

% for plotting
xArray    = [];
xhatArray = [];

for i=1:tf/dt
    
    if i>5000
        delta = 0.1;
    end
    
    xArray    = [xArray x];
    xhatArray = [xhatArray xhat];

    % Simulate the system
    x = eye(4)*x + [x(4)*cos(x(3))*dt;x(4)*sin(x(3))*dt;(x(4)/lf)*delta*dt;at*dt] + sqrt(Q)*[randn randn randn randn]'*dt;
    y = H*x + sqrt(R)*[randn randn]'*dt;

    % Calculate the Jacobian
    F = [1 0 -x(4)*sin(x(3))*dt cos(x(3))*dt;
         0 1 x(4)*cos(x(3))*dt sin(x(3))*dt;
         0 0 1 (delta*dt)/lf;
         0 0 0 1];
    
    % Prediction
    xmin  = F*xhat;
    Pmin  = F*Pplus*F' + Q;
    % Update
    K     = Pmin*H'*inv(H*Pmin*H' + R);
    Pplus = (eye(4)-K*H)*Pmin;
    xhat  = xmin + K*(y-H*xmin);
end

figure(1)
subplot(4,1,1)
plot(dt:dt:tf,xArray(1,:),'-b','LineWidth',3)
hold on;
plot(dt:dt:tf,xhatArray(1,:),':r','LineWidth',3)
legend('true state','estimated state')
set(gca,'FontSize',24)
grid on;
grid minor;
subplot(4,1,2)
plot(dt:dt:tf,xArray(2,:),'-b','LineWidth',3)
hold on;
plot(dt:dt:tf,xhatArray(2,:),':r','LineWidth',3)
legend('true state','estimated state')
set(gca,'FontSize',24)
grid on;
grid minor;
subplot(4,1,3)
plot(dt:dt:tf,xArray(3,:),'-b','LineWidth',3)
hold on;
plot(dt:dt:tf,xhatArray(3,:),':r','LineWidth',3)
legend('true state','estimated state')
set(gca,'FontSize',24)
grid on;
grid minor;
subplot(4,1,4)
plot(dt:dt:tf,xArray(4,:),'-b','LineWidth',3)
hold on;
plot(dt:dt:tf,xhatArray(4,:),':r','LineWidth',3)
legend('true state','estimated state')
set(gca,'FontSize',24)
grid on;
grid minor;

figure(2)
plot(xArray(1,:),xArray(2,:),'-b','LineWidth',3)
hold on;
plot(xhatArray(1,:),xhatArray(2,:),':r','LineWidth',3)