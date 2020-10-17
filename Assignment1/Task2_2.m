clear
clc
%% Loading dataset
load('GNSSINS.mat');
GNSS = in_data.GNSS.pos_ned(:,:);
ACC = in_data.IMU.acc(:,:)

%% Fixed values
dt = 1;
dtACC = 0.01;
tf = 299;

%% Covariance matrices
Q = 0.1*eye(3);
R = 0.1*eye(3);

%% System matrices
B = [1/2*dt 1/2*dt 1/2*dt]';
C = eye(3);

%% Initial estimates
xHat = [GNSS(1,1) GNSS(2,1) GNSS(3,1)]';
Pplus = 10000*eye(3);
Pmin = Pplus;

%% For plotting
xHatArray = [];

%% Kalmen filter
for i=1 : tf	
	y = [GNSS(1,i) GNSS(2,i) GNSS(3,i)];
	
	% Update Jacobian
	J = eye(3);
	
	% Predict step
	for j=1 : 100
		xHat = J * xHat + B.*ACC;
		Pmin = J * Pplus * J' + Q;
	end
	
	% Update step
	K = Pmin*C' * inv(C*Pmin*C' +R);
	xHat = xHat + K*(y' - C*xHat);
	Pplus = (eye(3)-K*C)*Pmin;
	xHatArray = [xHatArray xHat];
end

%% Plotting
figure(1)
subplot(3,1,1)
plot(dtACC:dtACC:tf,xHatArray(1,:),':r','LineWidth',3);
legend('GNSS X position');
xlabel('Sample time');
ylabel('position x');
hold on;
subplot(3,1,2)
plot(dt:dt:tf,xHatArray(2,:),':g','LineWidth',3);
legend('GNSS Y position');
xlabel('Sample time');
ylabel('position y');
hold on;
subplot(3,1,3)
plot(dt:dt:tf,xHatArray(3,:),':b','LineWidth',3);
legend('GNSS Z position');
xlabel('Sample time');
ylabel('position z');
hold on;