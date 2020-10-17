clear
clc
%% Tast 1 take 10

%% The kalman filter is correct - the math i wrong

%% Static data
m = 10;
dt = 0.001;
tf = 10;
load('DATA.mat');
[nn n] = size(xArray);

%% Covariance martices
Q = 0.1*eye(4);		% Covariance matrix for the model noice
R = 0.1*eye(2);		% Covariance matrix for the sensor noise

%% System matricesq
C = [1 0 0 0;
	 0 1 0 0];
B = [0; 1/m; 0; 0]*dt;
u = 10;

%% initial estimates
xHat = [xArray(1,1) xArray(2,1) 0 400]';
xHatm = xHat;
Pplus = 10000*eye(4);
Pmin = Pplus;

%% For plotting
xHatArray = [];

%% Kalman filter
for i=1:tf/dt
	
	% Input measurements
	y = [xArray(1,i); xArray(2,i)];
	
	% Update Jacobian
	J = [1 dt 0 0;
		-xHat(4)*dt/m 1-(xHat(3)*dt/m) -dt*xHat(2)/m -(xHat(1)*dt)/m;
		0 0 1 0;
		0 0 0 1];

	% predict
	%xHat = [xHat(1)+xHat(2)*dt; (-xHat(4)*dt/m)*xHat(1)+(1-(xHat(3)*dt/m))*xHat(2)+dt*u/m; xHat(3); xHat(4)];
	xHat = [xHat(1)+xHat(2)*dt; (-xHat(4)*dt/m)*xHat(1)+(1-(xHat(3)*dt/m))*xHat(2); xHat(3); xHat(4)] + B*u;
	%xHat = J * xHat + B*u; % 
	Pmin = J * Pplus* J' + Q;
	
	% Update
	K = Pmin * C' * inv(C * Pmin * C' + R);
	Pplus = (eye(4) - K * C) * Pmin;
	xHat = xHat + K * (y - C * xHat);
	
	% Write value to array for plotting
	xHatArray = [xHatArray xHat];
	
end

EstimatedValues =  xHatArray(:,10000)

%% Make graphs over estimated spring and damper coefficients
figure(1)
subplot(2,1,1)
plot(dt:dt:tf,xHatArray(4,:),':b','LineWidth',3);
legend('Estimated spring constant');
xlabel('Sample time');
ylabel('Spring coefficient value');
hold on;
subplot(2,1,2)
plot(dt:dt:tf,xHatArray(3,:),':r','LineWidth',3);
legend('Estimated dampning coefficient');
xlabel('Sample time');
ylabel('Damper coefficient value');

figure(2)
plot(dt:dt:tf,xArray(1,:),'-b','LineWidth',3);
hold on;
plot(dt:dt:tf,xHatArray(1,:),':r','LineWidth',3);
legend('Measured value of x','Estimated value of x');
xlabel('Sample time');
ylabel('x');