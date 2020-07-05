t = table2array(mpu6050a00(:,1));
ax = table2array(mpu6050a00(:,2));
ay = table2array(mpu6050a00(:,3));
az = table2array(mpu6050a00(:,4));
gx = table2array(mpu6050a00(:,5));
gy = table2array(mpu6050a00(:,6));
gz = table2array(mpu6050a00(:,7));

t = t.*(10^-6);

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