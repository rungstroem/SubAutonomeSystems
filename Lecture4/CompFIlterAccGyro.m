load('ExampleData.mat')
[n, ~] = size(time);

deltaT = 0;
alpha = 0.05;

for i = 1:n-1
    if(i-1 < 1)
        deltaT = 0;
    else
        deltaT = (time(i)-time(i-1));
    end
    gyro_ang_x = Gyroscope(i,1) * deltaT;
    gyro_ang_y = Gyroscope(i,2) * deltaT;
    gyro_ang_z = Gyroscope(i,3) * deltaT;
end

for i = 1:n-1
   acc_pos_x = atan(Accelerometer(i,2)/sqrt(Accelerometer(i,1)^2 + Accelerometer(i,3)^2));
   acc_pos_y = atan(Accelerometer(i,1)/sqrt(Accelerometer(i,2)^2 + Accelerometer(i,3)^2));
   acc_pos_z = atan(sqrt(Accelerometer(i,1)^2 + Accelerometer(i,2)^2)/Accelerometer(i,3));
end

for i = 1:n-1
   comp_x = gyro_ang_x*(1-alpha)+acc_pos_x*alpha;
   comp_y = gyro_ang_y*(1-alpha)+acc_pos_y*alpha;
   comp_z = gyro_ang_z*(1-alpha)+acc_pos_z*alpha;
end