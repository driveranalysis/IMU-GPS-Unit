%% Plotting raw data

close all;

% plotIMUdata(mpu6050a00);
% plotIMUdata(mpu6050a01);
% plotIMUdata(mpu6050a02);
plotIMUdata(mpu6050a03);
% plotIMUdata(mpu6050a04);
% plotIMUdata(mpu6050a19);



distFig('Rows',2,'Columns',3)
%% pre processing 

%PreprocessingIMU
datax = mpu6050a06;
t = table2array(datax(:,1));
ax = table2array(datax(:,2));
ay = table2array(datax(:,3));
az = table2array(datax(:,4));
gx = table2array(datax(:,5));
gy = table2array(datax(:,6));
gz = table2array(datax(:,7));

fs = 50;
AllData= [t ax ay az gx gy gz];
[acc_roll_error acc_pitch_error gyro_roll_error gyro_pitch_error gyro_yaw_error] = calculate_IMU_error(AllData);

ax = ax(201:end)/16384;
ay = ay(201:end)/16384;
az = az(201:end)/16384;
roll_acc = atan((ax) / sqrt(ax.^2 + az.^2) * 180 / pi)- acc_roll_error;
pitch_acc = atan(-ax/sqrt(ay.^2 + az.^2) *180/pi) - acc_pitch_error;

gx = gx(201:end)/131 - gyro_roll_error;
gy = gy(201:end)/131 - gyro_pitch_error;
gz = gz(201:end)/131 - gyro_yaw_error;

roll_gyro = zeros(length(gx));
pitch_gyro = zeros(length(gy));
yaw_gyro = zeros(length(gz));

for i= 1:length(gx)
roll_gyro(i+1) = roll_gyro(i) + gx(i)/(t(i+1)-t(i));
pitch_gyro(i+1) = pitch_gyro(i) +gy(i)/(t(i+1)-t(i));
yaw_gyro(i+1) = yaw_gyro(i) + gz(i)/(t(i+1)-t(i));
end


%%complimentary filter
roll = 0.96*roll_gyro +0.04*roll_acc;
pitch = 0.96*pitch_gyro +0.04*pitch_acc;
yaw = yaw_gyro;




% axf = ((ax(200:end)/16384.0))*9.81;
% ayf = ((ay(200:end)/16384.0))*9.81;
% azf = ((az(200:end)/16384.0))*9.81;
% gxf = ((gx(200:end)/131.0))*180/(2*pi);
% gyf = ((gy(200:end)/131.0))*180/(2*pi);
% gzf = ((gz(200:end)/131.0))*180/(2*pi);
% accelReadings = [axf ayf azf];
% gyroReadings = [gxf gyf gzf];
% azf = azf - 9.81; % assuming orientation remains the same
%% plotting using im fusion code

points = [2 1 1; 4 1 1; 4 2 1; 2 2 1; 2 2 2; 2 1 2; 4 1 2; 4 2 2];

faces = [1 2 3 4; 5 6 7 8; 2 3 8 7; 1 4 5 6; 1 2 7 6; 4 3 8 5];

figure
p1 = patch('faces',faces,...
    'vertices',points,...
    'facecolor','b',...
    'edgecolor',[1,1,1],...
    'facealpha',0.5);
p1.FaceAlpha    = 0.2;

xlim([0 5])
ylim([0 5])
zlim([-5 5])
grid on
view(3)

% rotation

for i= 1:length(yaw)

rotate(p1,[0,0,1],yaw,[3 1.5 1.5]);
rotate(p1,[1,0,0],roll,[3 1.5 1.5]);
rotate(p1,[0,1,0],pitch,[3 1.5 1.5]);
drawnow
end

%% calculating euler based on arduiino sketch





%% High pass and low pass filter
% Compute accelerometer magnitude
% acc_mag = sqrt(axf.*axf + ayf.*ayf + azf.*azf);
% 
% % HP filter accelerometer data
% filtCutOff = 0.001;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% acc_magFilt1 = filtfilt(b, a, acc_mag);
% acc_magFilt = filtfilt(b, a, acc_mag);
% 
% % Compute absolute value
% acc_magFilt = abs(acc_magFilt);
% 
% % LP filter accelerometer data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFilt2 = filtfilt(b, a, acc_magFilt);
% acc_magFilt = filtfilt(b, a, acc_magFilt);
% 
% % Threshold detection
% stationary = acc_magFilt < 0.05;
% 
% subplot 131
% plot(time,acc_mag);
% title('Original');
% subplot 132
% plot(time,acc_magFilt1);
% title('after highpass')
% subplot 133
% plot(time,acc_magFilt2);
% title('after lowpass');
% 
% correlation = xcorr(acc_magFilt1,acc_magFilt2);
% figure
% plot(correlation);
% title('Correlation of highpass and lowpass')
% 
% sub = abs(acc_magFilt1 - acc_magFilt2);
% figure
% plot(sub)
% title('difference between lowpass and highpass');
% distFig('rows',1,'columns',3);
%% AHRS

% AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);


%% Extracting orientationn and velocity using filter

% FUSE = imufilter;
% FUSE = imufilter('SampleRate',50);
% 
% [orientation,angularVelocity] = FUSE(accelReadings,gyroReadings);
% pose = FUSE(accelReadings,gyroReadings);
%% Plotting

%plotting orientation

% oPlotter = orientationPlotter(tp)
% 
% plotOrientation(oPlotter,orientation)



%quaternion to euler
eul = zeros(length(orientation),3);
for i= 1:length(orientation);
eul(i,1:3) = quat2eul(orientation(i));
end
%plotting using eul

points = [2 1 1; 4 1 1; 4 2 1; 2 2 1; 2 2 2; 2 1 2; 4 1 2; 4 2 2];

faces = [1 2 3 4; 5 6 7 8; 2 3 8 7; 1 4 5 6; 1 2 7 6; 4 3 8 5];

figure
p1 = patch('faces',faces,...
    'vertices',points,...
    'facecolor','b',...
    'edgecolor',[1,1,1],...
    'facealpha',0.5);
p1.FaceAlpha    = 0.2;

xlim([0 5])
ylim([0 5])
zlim([-5 5])
grid on
view(3)

vx = diff(axf)./0.02;
vy = diff(ayf)./0.02;
vz = diff(azf)./0.02;

sx = diff(axf,2)./0.02;
sy = diff(ayf,2)./0.02;
sz = diff(azf,2)./0.02;
% sx = resample(sx,
points_i = [2 1 1; 4 1 1; 4 2 1; 2 2 1; 2 2 2; 2 1 2; 4 1 2; 4 2 2];
% for i= 1:length(sx)
% sxi = sx(i);
% syi = sy(i);
% szi = sz(i);
% translation = repmat([sxi syi szi],8,1);
% points_i = points_i + translation;
% set(p1,'Vertices',points_i);
% drawnow
% end

% rotation

for i= 1:length(eul(:,1))
yaw = eul(i,1);
pitch = eul(i,2);
roll = eul(i,3);

rotate(p1,[0,0,1],yaw,[3 1.5 1.5]);
rotate(p1,[1,0,0],roll,[3 1.5 1.5]);
rotate(p1,[0,1,0],pitch,[3 1.5 1.5]);
% translate3d(p1,[i,i,i])
drawnow
end

%% Quiver plot

tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);
% for i=1:numel(pose)
%     plotOrientation(op, pose(i))
%     drawnow
% end

pt = [0 0 0];
dir = [1 0 0 1];
h = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3));
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

xfm = makehgtform('xrotate',pi/3,'yrotate',pi/5,'zrotate',pi/2);
newdir = xfm * dir';
h.UData = newdir(1);
h.VData = newdir(2);
h.WData = newdir(3);

for i= 1:length(eul)
      xfm = makehgtform('xrotate',eul(i,1),'yrotate',eul(i,2),'zrotate',eul(i,3));
      newdir = xfm * dir';
      h.UData = newdir(1);
      h.VData = newdir(2);
      h.WData = newdir(3);
      drawnow
      pause(2);
end



%% Writing data to serial port
for i= 1:length(eul(:,1));
myserialdevice = serialdev(mypi,'COM4');
write(myserialdevice,eul(i,:));
pause(2);
end


% plot(orientation);

%% Functions


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