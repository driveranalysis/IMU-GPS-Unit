## Python code for reading MPU6050 data saved to .csv file
import csv
import numpy as np
import matplotlib.pyplot as plt

plt.style.use('ggplot') # plot formatting

# .csv reader algorithm
csv_filename = 'mpu6050a00.csv'
data_headers = []
time_vec = []
accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z = [],[],[],[],[],[]
with open(csv_filename,newline='') as csvfile:
    csvreader = csv.reader(csvfile,delimiter=",")
    for row in csvreader:
        if data_headers==[]:
            data_headers = row
            continue
        time_vec.append(float(row[0]))
        accel_x.append(float(row[1]))
        accel_y.append(float(row[2]))
        accel_z.append(float(row[3]))
        gyro_x.append(float(row[4]))
        gyro_y.append(float(row[5]))
        gyro_z.append(float(row[6]))
        
time_vec = np.divide(time_vec,1000000.0) # correct for microseconds -> seconds

samp_rate = len(time_vec)/(time_vec[-1]-time_vec[0]) # psuedo sample rate
print('Sample Rate:  Hz'.format(samp_rate))

## conversion from bits to real-world values
accel_factor = ((2.0**15.0)-1.0)/2.0 # conversion using sensitivity (+- 2g)
gyro_factor = ((2.0**15.0)-1.0)/250.0 # conversion using sensitivity (250 deg/sec)

accel_x = np.array(accel_x)/accel_factor
accel_y = np.array(accel_y)/accel_factor
accel_z = np.array(accel_z)/accel_factor

gyro_x = np.array(gyro_x)/gyro_factor
gyro_y = np.array(gyro_y)/gyro_factor
gyro_z = np.array(gyro_z)/gyro_factor

## subplot with accel + gyro raw data after conversion
fig,ax = plt.subplots(2,1,figsize=(12,9))

ax1 = ax[0] # accel axis
cmap = plt.cm.Set1
ax1.plot(time_vec,accel_x,label='x-Acceleration',color=cmap(0),linewidth=4)
ax1.plot(time_vec,accel_y,label='y-Acceleration',color=cmap(1),linewidth=4)
ax1.plot(time_vec,accel_z,label='z-Acceleration',color=cmap(2),linewidth=4)
ax1.legend(fontsize=16)
ax1.set_ylabel('Acceleration [m $\cdot$ s$^$]',fontsize=16)

ax2 = ax[1] # gyro axis
cmap = plt.cm.tab10
ax2.plot(time_vec,gyro_x,label='x-gyroscope',color=cmap(1),linewidth=4)
ax2.plot(time_vec,gyro_y,label='y-gyroscope',color=cmap(4),linewidth=4)
ax2.plot(time_vec,gyro_z,label='z-gyroscope',color=cmap(6),linewidth=4)
ax2.legend(fontsize=16)
ax2.set_ylabel('Degrees [$^\circ$]',fontsize=16)
plt.xlabel('Time [s]',fontsize=16)
plt.savefig('mpu6050_subplot_accel_gyro_test.png',dpi=200,facecolor=[252/255,252/255,252/255]) # uncomment to save figure
plt.show()