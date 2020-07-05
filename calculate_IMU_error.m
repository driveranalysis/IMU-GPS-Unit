%%  Calculating IMU Bias
function [acc_roll_error acc_pitch_error gyro_roll_error gyro_pitch_error gyro_yaw_error] = calculate_IMU_error(AllData)
%   // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
%   // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
%   // Read accelerometer values 200 times

% AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
% AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
AccX = AllData(1:200,2)/16384;
AccY = AllData(1:200,3)/16384;
AccZ = AllData(1:200,4)/16384;

GyroX = AllData(1:200,5)/131;
GyroY = AllData(1:200,6)/131;
GyroZ = AllData(1:200,7)/131;

roll_acc = atan((AccY) / sqrt(AccX.^2 + AccZ.^2)*180/pi);
pitch_acc = atan(-AccX/sqrt(AccY.^2 + AccZ.^2) *180/pi);
acc_roll_error = mean(roll_acc(1:200));
acc_pitch_error = mean(pitch_acc(1:200));

gyro_roll_error = mean(GyroX);
gyro_pitch_error = mean(GyroY);
gyro_yaw_error = mean(GyroZ);


% MagX_error = mean(AllData(1:200,8));
% MagY_error = mean(AllData(1:200,9));
% MagZ_error = mean(AllData(1:200,10));

% AllError= [AccX_error AccY_error AccZ_error GyroX_error GyroY_error GyroZ_error]
end


% while (c < 200) {
%     Wire.beginTransmission(MPU);
%     Wire.write(0x3B);
%     Wire.endTransmission(false);
%     Wire.requestFrom(MPU, 6, true);
%     AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
%     AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
%     AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
%     // Sum all readings
%     AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
%     AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
%     c++;
%   end
%   //Divide the sum by 200 to get the error value
%   AccErrorX = AccErrorX / 200;
%   AccErrorY = AccErrorY / 200;
%   c = 0;
%   // Read gyro values 200 times
%   while (c < 200) {
%     Wire.beginTransmission(MPU);
%     Wire.write(0x43);
%     Wire.endTransmission(false);
%     Wire.requestFrom(MPU, 6, true);
%     GyroX = Wire.read() << 8 | Wire.read();
%     GyroY = Wire.read() << 8 | Wire.read();
%     GyroZ = Wire.read() << 8 | Wire.read();
%     // Sum all readings
%     GyroErrorX = GyroErrorX + (GyroX / 131.0);
%     GyroErrorY = GyroErrorY + (GyroY / 131.0);
%     GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
%     c++;
%   end
%   //Divide the sum by 200 to get the error value
%   GyroErrorX = GyroErrorX / 200;
%   GyroErrorY = GyroErrorY / 200;
%   GyroErrorZ = GyroErrorZ / 200;
% end