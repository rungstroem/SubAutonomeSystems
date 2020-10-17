clear
clc

%% Load data
load DATA.mat

%% Static parameters
m = 10;
deltaT = 0.001;
tf = 10;

%% System description matrices
B = [0 -deltaT/m 0 0]';
C = [1 0 0 0; 0 1 0 0];	%Usually called [H Matrix]
Cy = [1 0; 0 1];	%Matrix for output equation : y = C*x
u = 0;

%% Error covariance matrices
R = 0.1*eye(2);
Q = 0.1*eye(4);
Pplus = 100*eye(4);

%% Initial guess on state vector
xhatP = [20 0 0 0]';	%x(1) = pos, x(2) = vel, x(3) = DCoef, x(4) = SCoef

%% Jacobian matrix - declared and updated in for-loop
%J = [1 deltaT 0 0; -((xhat(4)*deltaT)/m) (1-(xhat(3)*deltaT)/m) -((deltaT*xhat(2))/m) -((xhat(1)*deltaT)/m); 0 0 1 0; 0 0 0 1];

%% For plotting
xhatArray = [];	
SCoef = [];		%Spring coefficient array
DCoef = [];		%Damper coefficient array

%% The Kalman-filter
for i=1 : length(xArray)
	y = Cy*xArray(i);	%Introduce data from file to the algorithm
	
	% Predicting
	J = [1 deltaT 0 0; -((xhatP(4)*deltaT)/m) (1-(xhatP(3)*deltaT)/m) -((deltaT*xhatP(2))/m) -((xhatP(1)*deltaT)/m); 0 0 1 0; 0 0 0 1]; %The Jacobian
	xhatM = J*xhatP+B*u;	%Make a prior estimate
	Pmin = J*Pplus*J'*Q;	%Make covariance matrix corresponding to a prior estimate

	% Update
	K = Pmin*C'*inv((C*Pmin*C'+R));	%Update kalman gain
	Pplus = (eye(4)-K*C)*Pmin;			%Calc. covariance matrix for a posterier estimate
	xhatP = xhatM + K*(y-C*xhatM);		%Calc. a posterier estimate
	
	SCoef = [SCoef xhatP(4)];	%Save sequence of estimated spring coefficients
	DCoef = [DCoef xhatP(3)];	%Save sequemce of estimated damper coefficeints
	
end

Spring_Coefficient = SCoef(10000)
Damper_Coefficient = DCoef(10000)

%% Make graphs over estimated spring and damper coefficients
%figure(1)
%subplot(2,1,1)
%plot(deltaT:deltaT:tf,SCoef(1,:),':b','LineWidth',3);
%legend('Estimated spring constant');
%xlabel('Sample time');
%ylabel('Spring coefficient value');
%hold on;
%subplot(2,1,2)
%plot(deltaT:deltaT:tf,DCoef(1,:),':r','LineWidth',3)
%legend('Estimated dampning coefficient');
%xlabel('Sample time');
%ylabel('Damper coefficient value');