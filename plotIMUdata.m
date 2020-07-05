%% Plotting IMU data from MPU6050 (no magnetometer)
% input= imported nx7 csv from imu
function [] = plotIMUdata(data)
    t = table2array(data(:,1));
    ax = table2array(data(:,2));
    ay = table2array(data(:,3));
    az = table2array(data(:,4));
    gx = table2array(data(:,5));
    gy = table2array(data(:,6));
    gz = table2array(data(:,7));

    ax = (ax/16384.0)*9.81;
    ay = (ay/16384.0)*9.81;
    az = (az/16384.0)*9.81;
    gx = (gx/131.0)*180/(2*pi);
    gy = (gy/131.0)*180/(2*pi);
    gz = (gz/131.0)*180/(2*pi);
    

    
    t = t.*(10^-6);
    figure
    subplot 231
    plot(t,ax);
    title('Accelerometer X');
    subplot 232
    plot(t,ay);
    title('Accelerometer Y')
    subplot 233
    plot(t,az);
    title('Accelerometer Z')
    subplot 234
    plot(t,gx);
    title('Gyroscope X')
    subplot 235
    plot(t,gy);
    title('Gyroscope Y')
    subplot 236
    plot(t,gz);
    title('Gyroscope Z')
end