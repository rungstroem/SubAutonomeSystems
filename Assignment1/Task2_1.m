clear
clc
%% Loading dataset
load('GNSSINS.mat');
GNSS = in_data.GNSS.pos_ned(:,:);

%% Fixed values
dt = 1;
tf = 299;

%% Plotting
figure(1)
subplot(3,1,1)
plot(dt:dt:tf,GNSS(1,:),':r','LineWidth',3);
legend('GNSS X position');
xlabel('Sample time');
ylabel('position x');
hold on;
subplot(3,1,2)
plot(dt:dt:tf,GNSS(2,:),':g','LineWidth',3);
legend('GNSS Y position');
xlabel('Sample time');
ylabel('position y');
hold on;
subplot(3,1,3)
plot(dt:dt:tf,GNSS(3,:),':b','LineWidth',3);
legend('GNSS Z position');
xlabel('Sample time');
ylabel('position z');
hold on;