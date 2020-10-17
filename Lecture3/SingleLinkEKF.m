%% Complete line code with % ??????????????????????????????????????????????????????????????????

clear all;
clc;

%% Simulation time
tf  = 6;
dt  = 0.001;
t   = dt:dt:tf;

%% System Description (S. Ibrir, et al., Int. J. of Control, vol. 78,, pp. 385--395, 2005.)
F = [];     % ??????????????????????????????????????????????????????????????????
C = [];     % ??????????????????????????????????????????????????????????????????
D = [];     % ??????????????????????????????????????????????????????????????????

%% Discrete system
A = [];     % ??????????????????????????????????????????????????????????????????
B = [];     % ??????????????????????????????????????????????????????????????????

%% Noise
QF = 1*eye(rank(A));
RF = 4*eye(rank(C));

%% Initialization
x        = [0;1;2;0]; % True state
xhat     = [1;1;1;1]; % nonlinear observer
Pplus    = 1000*eye(rank(A));

%% Paramater
u     = 0.1;
d     = 3.33;

%% For plotting

xArray     = [];    
xhatArray  = [];

%% EKF algorithm
for i=1:(tf/dt)    
    xArray         = [xArray x];
    xhatArray      = [xhatArray xhat]; 
    
    % Simulate the system
    x = ;       % ??????????????????????????????????????????????????????????????????
    y = ;       % ??????????????????????????????????????????????????????????????????
    % predict
    xhat  = A*xhat+dt*[0;0;0;-d*sin(xhat(3))]+B*u;
    FX    = ;       % ??????????????????????????????????????????????????????????????????
    Pmin  = FX*Pplus*FX'+QF;
    % update
    KF    = Pmin*C'*inv(C*Pmin*C'+RF);
    xhat  = xhat+KF*(y-C*xhat);
    Pplus = (eye(rank(A))-KF*C)*Pmin;
end

%% Plotting
figure(1);
subplot(4,1,1)
plot(t,xArray(1,:), 'k', 'LineWidth', 3)
hold on
plot(t,xhatArray(1,:), 'r:', 'LineWidth', 3)
grid on;
grid minor
ylabel('x_1','FontSize',12)
set(gca,'FontSize',12)
h1 = legend('True State','Estimated State','FontSize',12);
subplot(4,1,2)
plot(t,xArray(2,:), 'k', 'LineWidth', 3)
hold on
plot(t,xhatArray(2,:), 'r:', 'LineWidth', 3)
grid on;
grid minor
ylabel('x_2','FontSize',12)
set(gca,'FontSize',12)
subplot(4,1,3)
plot(t,xArray(3,:), 'k', 'LineWidth', 3)
hold on
plot(t,xhatArray(3,:), 'r:', 'LineWidth', 3)
grid on;
grid minor
ylabel('x_3','FontSize',12)
set(gca,'FontSize',12)
subplot(4,1,4)
plot(t,xArray(4,:), 'k', 'LineWidth', 3)
hold on
plot(t,xhatArray(4,:), 'r:', 'LineWidth', 3)
grid on;
grid minor
ylabel('x_4','FontSize',12)
xlabel('Time (s)','FontSize',12)
set(gca,'FontSize',12)
set(h1, 'Position', [0.6, 0.7, .1, .1])